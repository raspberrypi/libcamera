
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * backend_prepare.cpp - PiSP Back End configuration generation
 */
#include "backend.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "common/logging.hpp"
#include "common/utils.hpp"
#include "tiling/types.hpp"

using namespace libpisp;

namespace
{

// Limit this to a sensible size
constexpr unsigned int MaxStripeHeight = 3072;
// Precision for the scaler blocks
constexpr unsigned int ScalePrecision = 12;
constexpr unsigned int PhasePrecision = 12;
constexpr unsigned int UnityScale = 1 << ScalePrecision;
constexpr unsigned int UnityPhase = 1 << PhasePrecision;
// PPF properties
constexpr unsigned int ResamplePrecision = 10;
constexpr unsigned int NumPhases = 16;
constexpr unsigned int NumTaps = 6;

void check_stride(pisp_image_format_config const &config)
{
	if (config.stride % PISP_BACK_END_OUTPUT_MIN_ALIGN || config.stride2 % PISP_BACK_END_OUTPUT_MIN_ALIGN)
		throw std::runtime_error("Output stride values not sufficiently aligned");

	if (PISP_IMAGE_FORMAT_WALLPAPER(config.format) && (config.stride % 128 || config.stride2 % 128))
		throw std::runtime_error("Wallpaper format should have 128-byte aligned rolls");

	pisp_image_format_config check = config;
	compute_stride_align(check, PISP_BACK_END_OUTPUT_MIN_ALIGN);
	if (check.stride > config.stride || check.stride2 > config.stride2)
	{
		PISP_LOG(fatal, "Strides should be at least " << check.stride << " and " << check.stride2 << " but are "
													  << config.stride << " and " << config.stride2);
	}
}

void finalise_bayer_rgb_inputs(pisp_image_format_config const &config)
{
	if (config.width < PISP_BACK_END_MIN_TILE_WIDTH || config.height < PISP_BACK_END_MIN_TILE_HEIGHT)
		throw std::runtime_error("finalise_bayer_rgb_inputs: input image too small");
}

void finalise_inputs(pisp_be_config &config)
{
	// Not so much finalising, just checking that input dimensions and strides are OK.
	if (config.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT)
	{
		if ((config.input_format.width & 1) || (config.input_format.height & 1))
			throw std::runtime_error("finalise_inputs: Bayer pipe image dimensions must be even");
		if (config.input_format.stride & 15)
			throw std::runtime_error("finalise_inputs: input stride should be at least 16-byte aligned");
	}
	else if (config.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT)
	{
		if (PISP_IMAGE_FORMAT_SAMPLING_420(config.input_format.format) && (config.input_format.width & 1))
			throw std::runtime_error("finalise_inputs: 420 input height must be even");
		else if ((PISP_IMAGE_FORMAT_SAMPLING_420(config.input_format.format) ||
				  PISP_IMAGE_FORMAT_SAMPLING_422(config.input_format.format)) &&
				 (config.input_format.width & 1))
			throw std::runtime_error("finalise_inputs: 420/422 input width must be even");
		if (PISP_IMAGE_FORMAT_WALLPAPER(config.input_format.format))
		{
			if ((config.input_format.stride & 127) || (config.input_format.stride2 & 127))
				throw std::runtime_error("finalise_inputs: wallpaper format strides must be at least 128-byte aligned");
		}
		else if ((config.input_format.stride & 15) || (config.input_format.stride2 & 15))
			throw std::runtime_error("finalise_inputs: input strides must be at least 16-byte aligned");
	}
}

void finalise_lsc(pisp_be_lsc_config &lsc, [[maybe_unused]] pisp_be_lsc_extra &lsc_extra, uint16_t width,
				  uint16_t height)
{
	// Just a warning that ACLS algorithms might want the grid calculations here to match the AWB/ACLS stats.
	static const int P = PISP_BE_LSC_STEP_PRECISION;

	if (lsc.grid_step_x == 0)
		lsc.grid_step_x = (PISP_BE_LSC_GRID_SIZE << P) / width;
	if (lsc.grid_step_y == 0)
		lsc.grid_step_y = (PISP_BE_LSC_GRID_SIZE << P) / height;

	PISP_ASSERT(lsc.grid_step_x * (width + lsc_extra.offset_x - 1) < (PISP_BE_LSC_GRID_SIZE << P));
	PISP_ASSERT(lsc.grid_step_y * (height + lsc_extra.offset_y - 1) < (PISP_BE_LSC_GRID_SIZE << P));
}

void finalise_cac(pisp_be_cac_config &cac, [[maybe_unused]] pisp_be_cac_extra &cac_extra, uint16_t width,
				  uint16_t height)
{
	static const int P = PISP_BE_CAC_STEP_PRECISION;

	if (cac.grid_step_x == 0)
		cac.grid_step_x = (PISP_BE_CAC_GRID_SIZE << P) / width;
	if (cac.grid_step_y == 0)
		cac.grid_step_y = (PISP_BE_CAC_GRID_SIZE << P) / height;

	PISP_ASSERT(cac.grid_step_x * (width + cac_extra.offset_x - 1) < (PISP_BE_CAC_GRID_SIZE << P));
	PISP_ASSERT(cac.grid_step_y * (height + cac_extra.offset_y - 1) < (PISP_BE_CAC_GRID_SIZE << P));
}

void finalise_resample(pisp_be_resample_config &resample, pisp_be_resample_extra &resample_extra, uint16_t width,
					   uint16_t height)
{
	uint32_t scale_factor_h = ((width - 1) << ScalePrecision) / (resample_extra.scaled_width - 1);
	uint32_t scale_factor_v = ((height - 1) << ScalePrecision) / (resample_extra.scaled_height - 1);

	if ((scale_factor_h < UnityScale / 16 || scale_factor_h >= 16 * UnityScale) ||
		(scale_factor_v < UnityScale / 16 || scale_factor_v >= 16 * UnityScale))
		throw std::runtime_error("finalise_resample: Invalid scaling factors (must be < 16x down/upscale).");

	resample.scale_factor_h = scale_factor_h;
	resample.scale_factor_v = scale_factor_v;
	// If the filter coefficients are unset we should probably copy in our "default ones".
}

void finalise_downscale(pisp_be_downscale_config &downscale, pisp_be_downscale_extra &downscale_extra, uint16_t width,
						uint16_t height)
{
	PISP_LOG(debug, "width " << width << " scaled_width " << downscale_extra.scaled_width);
	PISP_LOG(debug, "height " << height << " scaled_height " << downscale_extra.scaled_height);

	uint32_t scale_factor_h = (width << ScalePrecision) / (downscale_extra.scaled_width);
	uint32_t scale_factor_v = (height << ScalePrecision) / (downscale_extra.scaled_height);

	if ((scale_factor_h != UnityScale && (scale_factor_h < 2 * UnityScale || scale_factor_h > 8 * UnityScale)) ||
		(scale_factor_v != UnityScale && (scale_factor_v < 2 * UnityScale || scale_factor_v > 8 * UnityScale)))
		throw std::runtime_error("finalise_downscale: Invalid scaling factors (must be 1x or >= 2x && <= 8x).");

	downscale.scale_factor_h = scale_factor_h;
	downscale.scale_factor_v = scale_factor_v;
	downscale.scale_recip_h = (downscale_extra.scaled_width << ScalePrecision) / (width);
	downscale.scale_recip_v = (downscale_extra.scaled_height << ScalePrecision) / (height);

	PISP_LOG(debug, "scale_factor_h " << downscale.scale_factor_h << " scale_factor_v " << downscale.scale_factor_v);
	PISP_LOG(debug, "scale_recip_h " << downscale.scale_recip_h << " scale_recip_v " << downscale.scale_recip_v);
}

void finalise_decompression(pisp_be_config const &be_config)
{
	uint32_t fmt = be_config.input_format.format, bayer_enables = be_config.global.bayer_enables;

	if (PISP_IMAGE_FORMAT_COMPRESSED(fmt) && !(bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS))
		throw std::runtime_error("BackEnd::finalise: input compressed but decompression not enabled");

	if (!PISP_IMAGE_FORMAT_COMPRESSED(fmt) && (bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS))
		throw std::runtime_error("BackEnd::finalise: input uncompressed but decompression enabled");

	if ((bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS) && !PISP_IMAGE_FORMAT_BPS_8(fmt))
		throw std::runtime_error("BackEnd::finalise: compressed input is not 8bpp");
}

// TDN and Stitch I/O dimensions must match the input, though the format may differ.
static void check_rawio_format(pisp_image_format_config &fmt, uint16_t w, uint16_t h)
{
	if (fmt.width == 0 || fmt.height == 0)
	{
		fmt.width = w;
		fmt.height = h;
	}
	else if (fmt.width != w || fmt.height != h)
		throw std::runtime_error("BackEnd::finalise: Image dimensions do not match input");

	if (fmt.stride == 0)
		compute_stride(fmt);
	else
		check_stride(fmt);
}

void finalise_tdn(pisp_be_config &config)
{
	int tdn_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN;
	int tdn_input_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT;
	int tdn_decompress_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS;
	int tdn_compress_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_COMPRESS;
	int tdn_output_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT;
	uint32_t fmt = config.tdn_output_format.format;

	if (tdn_enabled && !tdn_output_enabled)
		throw std::runtime_error("BackEnd::finalise: TDN output not enabled when TDN enabled");

	if (PISP_IMAGE_FORMAT_COMPRESSED(fmt) && !tdn_compress_enabled)
		throw std::runtime_error("BackEnd::finalise: TDN output compressed but compression not enabled");

	if (!PISP_IMAGE_FORMAT_COMPRESSED(fmt) && tdn_compress_enabled)
		throw std::runtime_error("BackEnd::finalise: TDN output uncompressed but compression enabled");

	if (tdn_compress_enabled && !PISP_IMAGE_FORMAT_BPS_8(fmt))
		throw std::runtime_error("BackEnd::finalise: TDN output does not match compression mode");

	if (tdn_output_enabled)
          check_rawio_format(config.tdn_output_format, config.input_format.width, config.input_format.height);

	if (tdn_input_enabled)
		check_rawio_format(config.tdn_input_format, config.input_format.width, config.input_format.height);

	if (!tdn_enabled)
	{
		if (tdn_input_enabled)
			throw std::runtime_error("BackEnd::finalise: TDN input enabled but TDN not enabled");
		// I suppose there is a weird (and entirely pointless) case where TDN is not enabled but TDN output is, which we allow.
	}
	else if (config.tdn.reset)
	{
		if (tdn_input_enabled)
			throw std::runtime_error("BackEnd::finalise: TDN input enabled but TDN being reset");
	}
	else
	{
		if (!tdn_input_enabled)
			throw std::runtime_error("BackEnd::finalise: TDN input not enabled but TDN not being reset");
		// Make the TDN input match the output if it's unset. Usually this will be the sensible thing to do.
		if (config.tdn_input_format.width == 0 && config.tdn_input_format.height == 0)
			config.tdn_input_format = config.tdn_output_format;
		if (PISP_IMAGE_FORMAT_COMPRESSED(fmt) && !tdn_decompress_enabled)
			throw std::runtime_error("BackEnd::finalise: TDN input compressed but decompression not enabled");
		if (!PISP_IMAGE_FORMAT_COMPRESSED(fmt) && tdn_decompress_enabled)
			throw std::runtime_error("BackEnd::finalise: TDN input uncompressed but decompression enabled");
		if (tdn_compress_enabled && !PISP_IMAGE_FORMAT_BPS_8(fmt))
			throw std::runtime_error("BackEnd::finalise: TDN output does not match compression mode");
	}
}

void finalise_stitch(pisp_be_config &config)
{
	bool stitch_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH;
	bool stitch_input_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_INPUT;
	bool stitch_decompress_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS;
	bool stitch_compress_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_COMPRESS;
	bool stitch_output_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_OUTPUT;
	uint32_t input_fmt = config.stitch_input_format.format;
	uint32_t output_fmt = config.stitch_output_format.format;

	if (stitch_enabled != stitch_input_enabled)
		throw std::runtime_error("BackEnd::finalise: stitch and stitch_input should be enabled/disabled together");
	if (stitch_input_enabled && PISP_IMAGE_FORMAT_COMPRESSED(input_fmt) && !stitch_decompress_enabled)
		throw std::runtime_error("BackEnd::finalise: stitch output compressed but decompression not enabled");
	if (stitch_input_enabled && !PISP_IMAGE_FORMAT_COMPRESSED(input_fmt) && stitch_decompress_enabled)
		throw std::runtime_error("BackEnd::finalise: stitch output uncompressed but decompression enabled");
	if (stitch_output_enabled && PISP_IMAGE_FORMAT_COMPRESSED(output_fmt) && !stitch_compress_enabled)
		throw std::runtime_error("BackEnd::finalise: stitch output compressed but compression not enabled");
	if (stitch_output_enabled && !PISP_IMAGE_FORMAT_COMPRESSED(output_fmt) && stitch_compress_enabled)
		throw std::runtime_error("BackEnd::finalise: stitch output uncompressed but compression enabled");
	if (stitch_decompress_enabled && !PISP_IMAGE_FORMAT_BPS_8(input_fmt))
		throw std::runtime_error("BackEnd::finalise: stitch input does not match compression mode");
	if (stitch_compress_enabled && !PISP_IMAGE_FORMAT_BPS_8(output_fmt))
		throw std::runtime_error("BackEnd::finalise: stitch output does not match compression mode");

	if (stitch_output_enabled)
		check_rawio_format(config.stitch_output_format, config.input_format.width, config.input_format.height);

	if (stitch_input_enabled)
		check_rawio_format(config.stitch_input_format, config.input_format.width, config.input_format.height);

	// Compute the motion_threshold reciprocal if it hasn't been done.
	if (config.stitch.motion_threshold_recip == 0)
	{
		if (config.stitch.motion_threshold_256 == 0)
			config.stitch.motion_threshold_recip = 255;
		else
			// We round the result up where possible as the block may work (ever so slightly) better like this.
			config.stitch.motion_threshold_recip =
				std::min(255, (256 + (int)config.stitch.motion_threshold_256 - 1) / config.stitch.motion_threshold_256);
	}
}

void finalise_output(pisp_be_output_format_config &config)
{
	// If the high clipping bound is zero assume it wasn't set and the intention is that no clipping occurs.
	if (config.hi == 0)
		config.hi = 65535;
	if (config.hi2 == 0)
		config.hi2 = 65535;

	// Do some checking on output image dimensions and strides.
	if (config.image.width < PISP_BACK_END_MIN_TILE_WIDTH || config.image.height < PISP_BACK_END_MIN_TILE_HEIGHT)
		throw std::runtime_error("finalise_output: output image too small");

	if (PISP_IMAGE_FORMAT_SAMPLING_420(config.image.format) && (config.image.height & 1))
		throw std::runtime_error("finalise_output: 420 image height should be even");

	if ((PISP_IMAGE_FORMAT_SAMPLING_420(config.image.format) || PISP_IMAGE_FORMAT_SAMPLING_422(config.image.format)) &&
		!PISP_IMAGE_FORMAT_INTERLEAVED(config.image.format) && (config.image.width & 1))
		throw std::runtime_error("finalise_output: 420/422 image width should be even");

	if (PISP_IMAGE_FORMAT_WALLPAPER(config.image.format))
	{
		if ((config.image.stride & 127) || (config.image.stride2 & 127))
			throw std::runtime_error("finalise_output: wallpaper image stride should be at least 128-byte aligned");
	}
	else if ((config.image.stride & 15) || (config.image.stride2 & 15))
		throw std::runtime_error("finalise_output: image stride should be at least 16-byte aligned");
}

void check_tiles(TileArray const &tiles, uint32_t rgb_enables, unsigned int numBranches, unsigned int num_tiles,
				 TilingConfig const &tiling_config)
{
	for (unsigned int tile_num = 0; tile_num < num_tiles; tile_num++)
	{
		const pisp_tile &tile = tiles[tile_num];

		PISP_ASSERT(tile.input_width && tile.input_height); // zero inputs shouldn't be possible

		if (tile.input_width < PISP_BACK_END_MIN_TILE_WIDTH || tile.input_height < PISP_BACK_END_MIN_TILE_HEIGHT)
			throw std::runtime_error("Tile too small at input");

		for (unsigned int i = 0; i < numBranches; i++)
		{
			if ((rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(i)) == 0)
				continue;

			unsigned int width_after_crop = tile.input_width - tile.crop_x_start[i] - tile.crop_x_end[i];
			unsigned int height_after_crop = tile.input_height - tile.crop_y_start[i] - tile.crop_y_end[i];

			// A tile that gets cropped away completely can't produce output, and vice versa.
			PISP_ASSERT((width_after_crop * height_after_crop == 0) ==
						(tile.output_width[i] * tile.output_height[i] == 0));

			// A zero-sized tile is legitimate meaning "no output", but otherwise minimum tile sizes must be respected.
			if (width_after_crop && height_after_crop)
			{
				bool rh_edge = tile.output_offset_x[i] + tile.output_width[i] == tiling_config.output_image_size[i].dx;

				if (width_after_crop < PISP_BACK_END_MIN_TILE_WIDTH)
				{
					PISP_LOG(warning, "Tile narrow after crop: tile " << tile_num << " output " << i
						 << " input_width " << tile.input_width << " after_crop " << width_after_crop
						 << " crop start " << tile.crop_x_start[i] << " end " << tile.crop_x_end[i]);
					if (!rh_edge)
						throw std::runtime_error("Tile width too small after crop");
				}
				if (height_after_crop < PISP_BACK_END_MIN_TILE_HEIGHT)
					throw std::runtime_error("Tile height too small after crop");

				if (tile.resample_in_width[i] < PISP_BACK_END_MIN_TILE_WIDTH)
				{
					PISP_LOG(warning, "Tile narrow after downscale: tile " << tile_num << " output " << i
						 << " input_width " << tile.input_width << " after_crop " << width_after_crop
						 << " after downscale " << tile.resample_in_width[i]);
					if (!rh_edge)
						throw std::runtime_error("Tile width too small after downscale");
				}

				if (tile.resample_in_height[i] < PISP_BACK_END_MIN_TILE_HEIGHT)
					throw std::runtime_error("Tile height too small after downscale");

				if (!rh_edge && tile.output_width[i] < PISP_BACK_END_MIN_TILE_WIDTH)
					throw std::runtime_error("Tile width too small at output");
				if (tile.output_height[i] < PISP_BACK_END_MIN_TILE_HEIGHT)
					throw std::runtime_error("Tile height too small at output");
			}
		}
	}
}

unsigned int get_pixel_alignment(uint32_t format, int byte_alignment)
{
	int alignment_pixels = byte_alignment; // for 8bpp formats

	if (PISP_IMAGE_FORMAT_BPS_16(format))
		alignment_pixels = byte_alignment / 2;
	else if (PISP_IMAGE_FORMAT_BPS_10(format))
		alignment_pixels = byte_alignment * 3 / 4;
	else if (PISP_IMAGE_FORMAT_BPP_32(format))
		alignment_pixels = byte_alignment / 4;

	if (PISP_IMAGE_FORMAT_PLANAR(format) && !PISP_IMAGE_FORMAT_SAMPLING_444(format))
		alignment_pixels *= 2; // the UV planes in fully planar 420/422 output will have half the width
	else if (PISP_IMAGE_FORMAT_INTERLEAVED(format) &&
			 (PISP_IMAGE_FORMAT_SAMPLING_422(format) || PISP_IMAGE_FORMAT_SAMPLING_420(format)))
		alignment_pixels /= 2; // YUYV type outputs need only 8 pixels to make 16 bytes

	return alignment_pixels;
}

unsigned int lcm(int a, int b)
{
	int orig_a = a, orig_b = b, tmp;
	while (b)
		tmp = a % b, a = b, b = tmp;
	return orig_a / a * orig_b;
}

static tiling::Length2 calculate_input_alignment(pisp_be_config const &config)
{
	if (config.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT)
	{
		PISP_LOG(debug, "RGB input enabled");
		// Need 4 byte alignment AND even number of pixels. Height must be 2 row aligned only for 420 input.
		return tiling::Length2(lcm(get_pixel_alignment(config.input_format.format, PISP_BACK_END_INPUT_ALIGN), 2),
							   PISP_IMAGE_FORMAT_SAMPLING_420(config.input_format.format) ? 2 : 1);
	}

	uint32_t bayer_enables = config.global.bayer_enables;
	// For starters, we need 4 *byte* alignment (this automatically cover 2 *pixel* alignment for all the raw formats).
	int pixel_alignment = get_pixel_alignment(config.input_format.format, PISP_BACK_END_INPUT_ALIGN);

	// If any input is compressed, we need 8 *pixel* alignment.
	if (PISP_IMAGE_FORMAT_COMPRESSED(config.input_format.format) ||
		((bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT) &&
		 PISP_IMAGE_FORMAT_COMPRESSED(config.tdn_input_format.format)) ||
		((bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_INPUT) &&
		 PISP_IMAGE_FORMAT_COMPRESSED(config.stitch_input_format.format)))
		pixel_alignment = lcm(pixel_alignment, PISP_BACK_END_COMPRESSED_ALIGN);

	// If any of the Bayer outputs are enabled, those need 16 *byte* alignment. (This already covers the outputs being compressed.)
	if (bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT)
		pixel_alignment =
			lcm(pixel_alignment, get_pixel_alignment(config.tdn_output_format.format, PISP_BACK_END_OUTPUT_MIN_ALIGN));
	if (bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)
		pixel_alignment = lcm(pixel_alignment,
							  get_pixel_alignment(config.stitch_output_format.format, PISP_BACK_END_OUTPUT_MIN_ALIGN));

	return tiling::Length2(pixel_alignment, 2); // Bayer input rows always in pairs
}

static tiling::Length2 calculate_output_alignment(uint32_t format, int align = PISP_BACK_END_OUTPUT_MAX_ALIGN)
{
	int y_alignment = PISP_IMAGE_FORMAT_SAMPLING_420(format) ? 2 : 1;

	return tiling::Length2(get_pixel_alignment(format, align), y_alignment);
}

void calculate_input_addr_offset(int x, int y, pisp_image_format_config const &input_format, uint32_t *addr_offset,
								 uint32_t *addr_offset2 = nullptr)
{
	uint32_t offset2 = 0;

	compute_addr_offset(input_format, x, y, addr_offset, &offset2);
	if (addr_offset2)
		*addr_offset2 = offset2;
}

} // namespace

void BackEnd::finaliseConfig()
{
	uint32_t dirty_flags_bayer = be_config_extra_.dirty_flags_bayer &
								 be_config_.global.bayer_enables; // only finalise blocks that are dirty *and* enabled
	uint32_t dirty_flags_rgb = be_config_extra_.dirty_flags_rgb &
							   be_config_.global.rgb_enables; // only finalise blocks that are dirty *and* enabled

	if ((dirty_flags_bayer & PISP_BE_BAYER_ENABLE_INPUT) || (dirty_flags_rgb & PISP_BE_RGB_ENABLE_INPUT))
		finalise_bayer_rgb_inputs(be_config_.input_format);

	if (dirty_flags_bayer & PISP_BE_BAYER_ENABLE_INPUT)
		finalise_inputs(be_config_);

	if (dirty_flags_bayer & (PISP_BE_BAYER_ENABLE_INPUT | PISP_BE_BAYER_ENABLE_DECOMPRESS))
		finalise_decompression(be_config_);

	if ((be_config_extra_.dirty_flags_bayer &
		 (PISP_BE_BAYER_ENABLE_TDN | PISP_BE_BAYER_ENABLE_TDN_INPUT | PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS |
		  PISP_BE_BAYER_ENABLE_TDN_COMPRESS | PISP_BE_BAYER_ENABLE_TDN_OUTPUT)))
	{
		finalise_tdn(be_config_);
	}

	if (be_config_extra_.dirty_flags_bayer &
		(PISP_BE_BAYER_ENABLE_STITCH | PISP_BE_BAYER_ENABLE_STITCH_INPUT | PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS |
		 PISP_BE_BAYER_ENABLE_STITCH_COMPRESS | PISP_BE_BAYER_ENABLE_STITCH_OUTPUT))
	{
		finalise_stitch(be_config_);
	}

	if (dirty_flags_bayer & PISP_BE_BAYER_ENABLE_LSC)
		finalise_lsc(be_config_.lsc, be_config_extra_.lsc, be_config_.input_format.width,
					 be_config_.input_format.height);

	if (dirty_flags_bayer & PISP_BE_BAYER_ENABLE_CAC)
		finalise_cac(be_config_.cac, be_config_extra_.cac, be_config_.input_format.width,
					 be_config_.input_format.height);

	for (unsigned int j = 0; j < variant_.BackEndNumBranches(0); j++)
	{
		bool enabled = be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(j);

		if (enabled)
		{
			// crop is enabled when it contains non-zero width/height
			uint16_t w = be_config_extra_.crop[j].width ? be_config_extra_.crop[j].width
														: be_config_.input_format.width;
			uint16_t h = be_config_extra_.crop[j].width ? be_config_extra_.crop[j].height
														: be_config_.input_format.height;

			if (dirty_flags_rgb & PISP_BE_RGB_ENABLE_DOWNSCALE(j))
			{
				if (variant_.BackEndDownscalerAvailable(0, j))
					finalise_downscale(be_config_.downscale[j], be_config_extra_.downscale[j], w, h);
				else
					throw std::runtime_error("Downscale is not available in output branch " + std::to_string(j));
			}

			if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(j))
			{
				// If the downscale is enabled, we update the input width/height for the resample stage.
				w = be_config_extra_.downscale[j].scaled_width;
				h = be_config_extra_.downscale[j].scaled_height;
			}

			if (dirty_flags_rgb & PISP_BE_RGB_ENABLE_RESAMPLE(j))
				finalise_resample(be_config_.resample[j], be_config_extra_.resample[j], w, h);

			if (dirty_flags_rgb & PISP_BE_RGB_ENABLE_OUTPUT(j))
				finalise_output(be_config_.output_format[j]);
		}
	}
	// Finally check for a sane collection of enable bits.
	if (!((be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) || (be_config_.global.bayer_enables == 0)))
		throw std::runtime_error("BackEnd::finalise: Bayer input disabled but Bayer pipe active");

	if (!!(be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) +
			!!(be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) !=
		1)
		throw std::runtime_error("BackEnd::finalise: exactly one of Bayer and RGB inputs should be enabled");

	uint32_t output_enables = be_config_.global.bayer_enables &
							  (PISP_BE_BAYER_ENABLE_TDN_OUTPUT | PISP_BE_BAYER_ENABLE_STITCH_OUTPUT);
	for (unsigned int i = 0; i < variant_.BackEndNumBranches(0); i++)
		output_enables |= be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(i);

	if (output_enables == 0)
		throw std::runtime_error("BackEnd::finalise: PiSP not configured to do anything");
}

void BackEnd::updateSmartResize()
{
	std::string filter;

	// Look through the output branches adjusting the scaling blocks where "smart resizing"
	// has been requested.
	for (unsigned int i = 0; i < variant_.BackEndNumBranches(0); i++)
	{
		// First get the size of the input to the rescalers. The crops are zero when not in use.
		uint16_t input_width = be_config_extra_.crop[i].width;
		if (!input_width)
			input_width = be_config_.input_format.width;
		uint16_t input_height = be_config_extra_.crop[i].height;
		if (!input_height)
			input_height = be_config_.input_format.height;

		if ((smart_resize_dirty_ & (1 << i)) || (be_config_extra_.dirty_flags_extra & PISP_BE_DIRTY_CROP))
		{
			if (smart_resize_[i].width && smart_resize_[i].height)
			{
				uint16_t resampler_input_width = input_width;
				uint16_t resampler_input_height = input_height;
				uint16_t resampler_output_width = smart_resize_[i].width;
				uint16_t resampler_output_height = smart_resize_[i].height;

				PISP_LOG(debug, "Smart resize branch " << i
						 << " input size " << input_width << " x " << input_height
						 << " output size " << smart_resize_[i].width << " x " <<  smart_resize_[i].height);

				// We're doing to use the downscaler if it's available and we're downscaling
				// by more than 2x.
				// \todo - increase this "2x" threshold by using different resampler kernels.
				if (variant_.BackEndDownscalerAvailable(0, i) &&
					(resampler_output_width * 2 < input_width || resampler_output_height * 2 < input_height))
				{
					uint16_t downscaler_output_width = input_width;
					uint16_t downscaler_output_height = input_height;

					// We treat the width and height the same. Look at the width first.
					if (resampler_output_width * 2 < input_width)
					{
						// Try to put 2x downscale into the resampler, everything else into
						// the downscaler. But remember that it must do *at least* 2x, and no
						// more than 8x (being careful to round that limit up)..
						downscaler_output_width = std::clamp(resampler_output_width * 2,
															 (input_width + 7) / 8,
															 input_width / 2);
					}
					// Now the same for the height.
					if (resampler_output_height * 2 < input_height)
					{
						// Try to put 2x downscale into the resampler, everything else into
						// the downscaler. But remember that it must do *at least* 2x and no
						// more than 8x (being careful to round that limit up)..
						downscaler_output_height = std::clamp(resampler_output_height * 2,
															  (input_height + 7) / 8,
															  input_height / 2);
					}

					PISP_LOG(debug, "Using downscaler, output size "
							 << downscaler_output_width << " x " <<  downscaler_output_height);

					// Now program up the downscaler.
					pisp_be_downscale_extra downscale = {};
					downscale.scaled_width = downscaler_output_width;
					downscale.scaled_height = downscaler_output_height;
					SetDownscale(i, downscale);
					be_config_.global.rgb_enables |= PISP_BE_RGB_ENABLE_DOWNSCALE(i);

					// Now adjust the input dimensions that the code below (which
					// programs the resampler) will see, so that it can, if necessary,
					// do its PPF coefficient adjustment.
					resampler_input_width = downscaler_output_width;
					resampler_input_height = downscaler_output_height;
				}
				else
				{
					be_config_.global.rgb_enables &= ~PISP_BE_RGB_ENABLE_DOWNSCALE(i);
				}

				// Don't resample by unity: it needlessly reduces bit-depth, and can over-sharpen
				if (resampler_input_width == resampler_output_width &&
					resampler_input_height == resampler_output_height)
				{
					be_config_.global.rgb_enables &= ~PISP_BE_RGB_ENABLE_RESAMPLE(i);
					continue;
				}

				// Finally program up the resampler block.
				// If the following conditions are met:
				//
				// - The x and y scale factors are the same,
				// - We are downscaling in both directions by a factor of > 2,
				// - The downscale factor is *smaller* than the number of filter taps - 1,
				//
				// then we can use the PPF as a trapezoidal downscaler by setting up
				// the filter coefficients (per used phase) correctly. This improves
				// on image quality for larger downscale factors.
				double scale_factor_x = (double)(resampler_input_width - 1) / (resampler_output_width - 1);
				double scale_factor_y = (double)(resampler_input_height - 1) / (resampler_output_height - 1);
				pisp_be_resample_config resample = {};
				pisp_be_resample_extra resample_extra = {};

				if (scale_factor_x > 2.1 &&
					scale_factor_x < scale_factor_y * 1.1 && scale_factor_y < scale_factor_x * 1.1)
				{
					PISP_LOG(debug, "Setting the PPF as a trapezoidal filter");

					scale_factor_x = std::min<double>(scale_factor_x, NumTaps - 1);

					for (unsigned int p = 0; p < NumPhases; p++)
					{
						// Initial phase for the current pixel (offset 2 in the filter)
						// is calculated as 1 - p / NumPhases
						resample.coef[p * NumTaps + 0] =
							(1 << ResamplePrecision) - (p << ResamplePrecision) / NumPhases;

						resample.coef[p * NumTaps + 0] /= scale_factor_x;

						double scale = scale_factor_x - (1.0 - (double)p / NumPhases);
						for (unsigned int t = 1; t < 1 + std::ceil(scale_factor_x); t++)
						{
							double s = std::min(1.0, scale);
							resample.coef[p * NumTaps + t] = s * (1 << ResamplePrecision) / scale_factor_x;
							scale -= s;
						}
					}

					// resample_extra will be filled in below.
					SetResample(i, resample, resample_extra);
				}
				else
				{
					// Let's choose a resampling filter based on the scaling factor.
					// The selection mapping is defined in the config json file.
					InitialiseResample(resample, scale_factor_x);
				}

				// Last thing is to set the output dimensions.
				resample_extra.scaled_width = resampler_output_width;
				resample_extra.scaled_height = resampler_output_height;
				SetResample(i, resample_extra);
				be_config_.global.rgb_enables |= PISP_BE_RGB_ENABLE_RESAMPLE(i);
			}
		}
	}

	smart_resize_dirty_ = 0;
}

void BackEnd::updateTiles()
{
	if (retile_)
	{
		TilingConfig tiling_config;
		pisp_be_config const &c = be_config_;
		BeConfigExtra const &ce = be_config_extra_;

		retile_ = false;
		tiling_config.input_alignment = calculate_input_alignment(c);

		PISP_LOG(debug, "Input alignments are " << tiling_config.input_alignment << " pixels");

		tiling_config.input_image_size = tiling::Length2(c.input_format.width, c.input_format.height);

		for (unsigned int i = 0; i < variant_.BackEndNumBranches(0); i++)
		{
			tiling_config.crop[i] = tiling::Interval2(tiling::Interval(ce.crop[i].offset_x, ce.crop[i].width),
													  tiling::Interval(ce.crop[i].offset_y, ce.crop[i].height));

			if (tiling_config.crop[i].x.length == 0 || tiling_config.crop[i].y.length == 0)
				tiling_config.crop[i] = tiling::Interval2(tiling::Interval(0, c.input_format.width),
														  tiling::Interval(0, c.input_format.height));

			tiling_config.output_h_mirror[i] = be_config_.output_format[i].transform & PISP_BE_TRANSFORM_HFLIP;
			tiling_config.downscale_factor[i] =
				tiling::Length2(c.downscale[i].scale_factor_h, c.downscale[i].scale_factor_v);
			tiling_config.resample_factor[i] =
				tiling::Length2(c.resample[i].scale_factor_h, c.resample[i].scale_factor_v);
			tiling_config.downscale_image_size[i] =
				tiling::Length2(ce.downscale[i].scaled_width, ce.downscale[i].scaled_height);
			tiling_config.output_image_size[i] =
				tiling::Length2(c.output_format[i].image.width, c.output_format[i].image.height);
			tiling_config.output_max_alignment[i] =
				calculate_output_alignment(c.output_format[i].image.format, PISP_BACK_END_OUTPUT_MAX_ALIGN);
			tiling_config.output_min_alignment[i] =
				calculate_output_alignment(c.output_format[i].image.format, PISP_BACK_END_OUTPUT_MIN_ALIGN);
		}

		tiling_config.max_tile_size.dx = config_.max_tile_width ? config_.max_tile_width
																: variant_.BackEndMaxTileWidth(0);
		tiling_config.max_tile_size.dy = config_.max_stripe_height ? config_.max_stripe_height : MaxStripeHeight;
		tiling_config.min_tile_size = tiling::Length2(PISP_BACK_END_MIN_TILE_WIDTH, PISP_BACK_END_MIN_TILE_HEIGHT);
		tiling_config.resample_enables = be_config_.global.rgb_enables / (int)PISP_BE_RGB_ENABLE_RESAMPLE0;
		tiling_config.downscale_enables = be_config_.global.rgb_enables / (int)PISP_BE_RGB_ENABLE_DOWNSCALE0;

		// Set compressed_input to false as otherwise the tiling would pad tiles up to multiples of 8 pixels even when these lie
		// outside the actual image width (and we've chosen not to handle compression like that).
		tiling_config.compressed_input = false;
		tiles_ = retilePipeline(tiling_config);
		check_tiles(tiles_, c.global.rgb_enables, variant_.BackEndNumBranches(0), num_tiles_x_ * num_tiles_y_,
					tiling_config);
		finalise_tiling_ = true;
	}

	if (finalise_tiling_)
	{
		finaliseTiling();
		finalise_tiling_ = false;
	}
}

TileArray BackEnd::retilePipeline(TilingConfig const &tiling_config)
{
	// The tiling library provides tiles in a SW Tile structure.
	Tile tiles[PISP_BACK_END_NUM_TILES];
	tiling::Length2 grid;

	tile_pipeline(tiling_config, tiles, PISP_BACK_END_NUM_TILES, &grid);
	num_tiles_x_ = grid.dx;
	num_tiles_y_ = grid.dy;

	TileArray tile_array;
	// Finally convert the Tiles into pisp_tiles.
	for (int i = 0; i < num_tiles_x_ * num_tiles_y_; i++)
	{
		pisp_tile &t = tile_array[i];

		memset(&t, 0, sizeof(pisp_tile));
		t.edge = 0;
		if (i < num_tiles_x_)
			t.edge |= PISP_TOP_EDGE;
		if (i >= num_tiles_x_ * (num_tiles_y_ - 1))
			t.edge |= PISP_BOTTOM_EDGE;
		if (i % num_tiles_x_ == 0)
			t.edge |= PISP_LEFT_EDGE;
		if ((i + 1) % num_tiles_x_ == 0)
			t.edge |= PISP_RIGHT_EDGE;

		t.input_offset_x = tiles[i].input.input.x.offset;
		t.input_offset_y = tiles[i].input.input.y.offset;
		t.input_width = tiles[i].input.input.x.length;
		t.input_height = tiles[i].input.input.y.length;

		if (tiles[i].input.output != tiles[i].input.input)
			throw std::runtime_error("BackEnd::retilePipeline: tiling error in Bayer pipe");

		for (unsigned int j = 0; j < variant_.BackEndNumBranches(0); j++)
		{
			bool enabled = (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(j));

			if (enabled && (tiles[i].output[j].output.x.length == 0 || tiles[i].output[j].output.y.length == 0))
			{
				// If a tile produces no output there's no point sending anything down this branch, so ensure the crop
				// "eats" everything and set everything else to zero.
				t.crop_x_start[j] = t.input_width;
				t.crop_x_end[j] = 0;
				t.crop_y_start[j] = t.input_height;
				t.crop_y_end[j] = 0;
				t.resample_in_width[j] = 0;
				t.resample_in_height[j] = 0;
				t.output_offset_x[j] = 0;
				t.output_offset_y[j] = 0;
				t.output_width[j] = 0;
				t.output_height[j] = 0;
				continue;
			}

			tiling::Crop2 downscale_crop;
			tiling::Interval2 resample_size = tiles[i].crop[j].output;
			resample_size.x = resample_size.x - tiles[i].resample[j].crop.x;
			resample_size.y = resample_size.y - tiles[i].resample[j].crop.y;

			// When a resize stage is disabled, the tile size after the stage is found from the input of the
			// next block. Also there will be no extra crop necessary for the resize operation.
			if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(j))
			{
				downscale_crop = tiles[i].downscale[j].crop + tiles[i].crop[j].crop;
				// Size of the tile going into the resample block needs to be set here.
				resample_size = tiles[i].downscale[j].output;
			}
			else if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(j))
			{
				downscale_crop = tiles[i].resample[j].crop + tiles[i].crop[j].crop;
			}
			else
			{
				downscale_crop = tiles[i].output[j].crop + tiles[i].crop[j].crop;
			}

			t.crop_x_start[j] = downscale_crop.x.start;
			t.crop_x_end[j] = downscale_crop.x.end;
			t.crop_y_start[j] = downscale_crop.y.start;
			t.crop_y_end[j] = downscale_crop.y.end;
			t.resample_in_width[j] = resample_size.x.length;
			t.resample_in_height[j] = resample_size.y.length;
			t.output_offset_x[j] = tiles[i].output[j].output.x.offset;
			t.output_offset_y[j] = tiles[i].output[j].output.y.offset;
			t.output_width[j] = tiles[i].output[j].output.x.length;
			t.output_height[j] = tiles[i].output[j].output.y.length;

			for (int p = 0; p < 3; p++)
			{
				// Calculate x/y initial downsampler/resampler phases per-plane.
				if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(j))
				{
					unsigned int frac_x = (resample_size.x.offset * be_config_.downscale[j].scale_factor_h) &
										  ((1 << ScalePrecision) - 1);
					unsigned int frac_y = (resample_size.y.offset * be_config_.downscale[j].scale_factor_v) &
										  ((1 << ScalePrecision) - 1);
					// Fractional component of the input required to generate the output pixel.
					t.downscale_phase_x[p * variant_.BackEndNumBranches(0) + j] = (UnityPhase - frac_x);
					t.downscale_phase_y[p * variant_.BackEndNumBranches(0) + j] = (UnityPhase - frac_y);
				}

				if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(j))
				{
					// Location of the output pixel in the interpolated (input) image.
					unsigned int interpolated_pix_x =
						(t.output_offset_x[j] * NumPhases * be_config_.resample[j].scale_factor_h) >> ScalePrecision;
					unsigned int interpolated_pix_y =
						(t.output_offset_y[j] * NumPhases * be_config_.resample[j].scale_factor_v) >> ScalePrecision;
					// Phase of the interpolated input pixel.
					t.resample_phase_x[p * variant_.BackEndNumBranches(0) + j] =
						((interpolated_pix_x % NumPhases) << ScalePrecision) / NumPhases;
					t.resample_phase_y[p * variant_.BackEndNumBranches(0) + j] =
						((interpolated_pix_y % NumPhases) << ScalePrecision) / NumPhases;
					// Account for any user defined initial phase - this could be negative!
					t.resample_phase_x[p * variant_.BackEndNumBranches(0) + j] +=
						be_config_extra_.resample[j].initial_phase_h[p];
					t.resample_phase_y[p * variant_.BackEndNumBranches(0) + j] +=
						be_config_extra_.resample[j].initial_phase_v[p];
					// Have to be within this range, else some calculation went wrong.
					PISP_ASSERT(t.resample_phase_x[p * variant_.BackEndNumBranches(0) + j] <= (2 * UnityPhase - 1));
					PISP_ASSERT(t.resample_phase_y[p * variant_.BackEndNumBranches(0) + j] <= (2 * UnityPhase - 1));
				}
			}

			// Phase difference between planes cannot be > 0.5 pixels on the output dimenstions.
			if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(j))
			{
				int phase_max = (be_config_.resample[j].scale_factor_h * UnityPhase / 2) >> ScalePrecision;
				if (std::abs(t.resample_phase_x[0 * variant_.BackEndNumBranches(0) + j] -
							 t.resample_phase_x[1 * variant_.BackEndNumBranches(0) + j]) > phase_max ||
					std::abs(t.resample_phase_x[1 * variant_.BackEndNumBranches(0) + j] -
							 t.resample_phase_x[2 * variant_.BackEndNumBranches(0) + j]) > phase_max ||
					std::abs(t.resample_phase_x[0 * variant_.BackEndNumBranches(0) + j] -
							 t.resample_phase_x[2 * variant_.BackEndNumBranches(0) + j]) > phase_max)
				{
					throw std::runtime_error("Resample phase x for tile is > 0.5 pixels on the output dimensions.");
				}
				phase_max = (be_config_.resample[j].scale_factor_v * UnityPhase / 2) >> ScalePrecision;
				if (std::abs(t.resample_phase_y[0 * variant_.BackEndNumBranches(0) + j] -
							 t.resample_phase_y[1 * variant_.BackEndNumBranches(0) + j]) > phase_max ||
					std::abs(t.resample_phase_y[1 * variant_.BackEndNumBranches(0) + j] -
							 t.resample_phase_y[2 * variant_.BackEndNumBranches(0) + j]) > phase_max ||
					std::abs(t.resample_phase_y[0 * variant_.BackEndNumBranches(0) + j] -
							 t.resample_phase_y[2 * variant_.BackEndNumBranches(0) + j]) > phase_max)
				{
					throw std::runtime_error("Resample phase y for tile is > 0.5 pixels on the output dimensions.");
				}
			}
		}
	}
	return tile_array;
}

void BackEnd::finaliseTiling()
{
	// Update tile parameters (offsets/strides) from on the BE pipeline configuration.
	for (int i = 0; i < num_tiles_x_ * num_tiles_y_; i++)
	{
		pisp_tile &t = tiles_[i];

		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.input_format, &t.input_addr_offset,
									&t.input_addr_offset2);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.tdn_input_format,
									&t.tdn_input_addr_offset);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.tdn_output_format,
									&t.tdn_output_addr_offset);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.stitch_input_format,
									&t.stitch_input_addr_offset);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.stitch_output_format,
									&t.stitch_output_addr_offset);
		PISP_LOG(debug, "Input offsets " << t.input_offset_x << "," << t.input_offset_y << " address offsets "
										 << t.input_addr_offset << " and " << t.input_addr_offset2);

		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_LSC)
		{
			t.lsc_grid_offset_x = (t.input_offset_x + be_config_extra_.lsc.offset_x) * be_config_.lsc.grid_step_x;
			t.lsc_grid_offset_y = (t.input_offset_y + be_config_extra_.lsc.offset_y) * be_config_.lsc.grid_step_y;
		}

		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC)
		{
			t.cac_grid_offset_x = (t.input_offset_x + be_config_extra_.cac.offset_x) * be_config_.cac.grid_step_x;
			t.cac_grid_offset_y = (t.input_offset_y + be_config_extra_.cac.offset_y) * be_config_.cac.grid_step_y;
		}

		for (unsigned int j = 0; j < variant_.BackEndNumBranches(0); j++)
		{
			int output_offset_x_unflipped = t.output_offset_x[j], output_offset_y_unflipped = t.output_offset_y[j];

			if (be_config_.output_format[j].transform & PISP_BE_TRANSFORM_HFLIP)
				t.output_offset_x[j] =
					be_config_.output_format[j].image.width - output_offset_x_unflipped - t.output_width[j];

			if (be_config_.output_format[j].transform & PISP_BE_TRANSFORM_VFLIP)
				t.output_offset_y[j] = be_config_.output_format[j].image.height - output_offset_y_unflipped - 1;

			compute_addr_offset(be_config_.output_format[j].image, t.output_offset_x[j], t.output_offset_y[j],
								&t.output_addr_offset[j], &t.output_addr_offset2[j]);

			PISP_LOG(debug, "Branch " << j << " output offsets " << t.output_offset_x[j] << "," << t.output_offset_y[j]
									  << " address offsets " << t.output_addr_offset[j] << " and "
									  << t.output_addr_offset2[j]);
		}
	}
}

void BackEnd::getOutputSize(int i, uint16_t *width, uint16_t *height, pisp_image_format_config const &ifmt) const
{
	// This internal version doesn't check if the output is enabled
	if (smart_resize_[i].width && smart_resize_[i].height)
		*width = smart_resize_[i].width, *height = smart_resize_[i].height;
	else if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(i))
		*width = be_config_extra_.resample[i].scaled_width, *height = be_config_extra_.resample[i].scaled_height;
	else if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(i))
		*width = be_config_extra_.downscale[i].scaled_width, *height = be_config_extra_.downscale[i].scaled_height;
	else if (be_config_extra_.crop[i].width) // crop width and height will be zero when crop disabled
		*width = be_config_extra_.crop[i].width, *height = be_config_extra_.crop[i].height;
	else
		*width = ifmt.width, *height = ifmt.height;
}

bool BackEnd::ComputeOutputImageFormat(unsigned int i, pisp_image_format_config &fmt,
									   pisp_image_format_config const &ifmt) const
{
	PISP_ASSERT(i < PISP_BACK_END_NUM_OUTPUTS);

	if (&fmt != &be_config_.output_format[i].image)
		fmt = be_config_.output_format[i].image;

	if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(i))
	{
		getOutputSize(i, &fmt.width, &fmt.height, ifmt);
		if (!fmt.stride)
			compute_stride(fmt);
		else
			check_stride(fmt);
		return true;
	}
	else
	{
		fmt.width = 0;
		fmt.height = 0;
		fmt.stride = 0;
		fmt.stride2 = 0;
		return false;
	}
}

void BackEnd::Prepare(pisp_be_tiles_config *config)
{
	PISP_LOG(debug, "New frame!");

	// On every start-of-frame we:
	// 1. Check the input configuration appears sensible.
	if ((be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) == 0 &&
		(be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) == 0)
		throw std::runtime_error("BackEnd::preFrameUpdate: neither Bayer nor RGB inputs are enabled");
	else if ((be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) &&
			 (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT))
		throw std::runtime_error("BackEnd::preFrameUpdate: both Bayer and RGB inputs are enabled");

	// 2. Also check the output configuration is all filled in and looks sensible. Again, addresses must be
	// left to the HAL.
	for (unsigned int i = 0; i < variant_.BackEndNumBranches(0); i++)
	{
		pisp_image_format_config &image_config = be_config_.output_format[i].image;
		ComputeOutputImageFormat(i, image_config, be_config_.input_format);

		if (image_config.format & PISP_IMAGE_FORMAT_INTEGRAL_IMAGE)
		{
			throw std::runtime_error("Integral images are not supported.");
		}
	}

	// 3. Fill in any other missing bits of config, and update the tiling if necessary.
	updateSmartResize();
	finaliseConfig();
	updateTiles();

	if (config)
	{ // Allow passing of empty pointer, if only be_config_ should be filled
		// 4. Write the config and tiles to the provided buffer to send to the hardware.
		config->num_tiles = num_tiles_x_ * num_tiles_y_;
		memcpy(config->tiles, tiles_.data(), config->num_tiles * sizeof(pisp_tile));
		config->config = be_config_;

		// 5. Clear any dirty flags for the next configuration update.
		be_config_extra_.dirty_flags_bayer = be_config_extra_.dirty_flags_rgb = be_config_extra_.dirty_flags_extra = 0;
	}
}
