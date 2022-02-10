#include "backend.h"

#include <libcamera/base/log.h>

#include "../common/utils.h"
#include "tiling/types.h"

using namespace libcamera;
using namespace PiSP;

LOG_DECLARE_CATEGORY(PISPBE)

namespace {

// Limit this to a sensible size
constexpr unsigned int MaxStripeHeight = 3072;
// Precision for the scaler blocks
constexpr unsigned int ScalePrecision = 12;
constexpr unsigned int UnityScale = 1 << 12;
// HoG feature constants
constexpr unsigned int HogCellSize = 8;

void finalise_bayer_rgb_inputs(pisp_image_format_config const &config)
{
	if (config.width < PISP_BACK_END_MIN_TILE_WIDTH || config.height < PISP_BACK_END_MIN_TILE_HEIGHT)
		LOG(PISPBE, Error) << "finalise_bayer_rgb_inputs: input image too small";
}

void finalise_inputs(pisp_be_config &config)
{
	// Not so much finalising, just checking that input dimensions and strides are OK.
	if (config.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) {
		if ((config.input_format.width & 1) || (config.input_format.height & 1))
			LOG(PISPBE, Error) << "finalise_inputs: Bayer pipe image dimensions must be even";
		if (config.input_format.stride & 15)
			LOG(PISPBE, Error) << "finalise_inputs: input stride should be at least 16-byte aligned";
	} else if (config.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) {
		if (PISP_IMAGE_FORMAT_sampling_420(config.input_format.format) && (config.input_format.width & 1))
			LOG(PISPBE, Error) << "finalise_inputs: 420 input height must be even";
		else if ((PISP_IMAGE_FORMAT_sampling_420(config.input_format.format) || PISP_IMAGE_FORMAT_sampling_422(config.input_format.format)) && (config.input_format.width & 1))
			LOG(PISPBE, Error) << "finalise_inputs: 420/422 input width must be even";
		if (PISP_IMAGE_FORMAT_wallpaper(config.input_format.format)) {
			if ((config.input_format.stride & 127) || (config.input_format.stride2 & 127))
				LOG(PISPBE, Error) << "finalise_inputs: wallpaper format strides must be at least 128-byte aligned";
		} else if ((config.input_format.stride & 15) || (config.input_format.stride2 & 15))
			LOG(PISPBE, Error) << "finalise_inputs: input strides must be at least 16-byte aligned";
	}
}

void finalise_lsc(pisp_be_lsc_config &lsc, pisp_be_lsc_extra &lsc_extra, uint16_t width, uint16_t height)
{
	// Just a warning that ACLS algorithms might want the grid calculations here to match the AWB/ACLS stats.
	static const int P = PISP_BE_LSC_STEP_PRECISION;

	if (lsc.grid_step_x == 0)
		lsc.grid_step_x = (PISP_BE_LSC_GRID_SIZE << P) / width;
	if (lsc.grid_step_y == 0)
		lsc.grid_step_y = (PISP_BE_LSC_GRID_SIZE << P) / height;

	ASSERT(lsc.grid_step_x * (width + lsc_extra.offset_x - 1) < (PISP_BE_LSC_GRID_SIZE << P));
	ASSERT(lsc.grid_step_y * (height + lsc_extra.offset_y - 1) < (PISP_BE_LSC_GRID_SIZE << P));
}

void finalise_cac(pisp_be_cac_config &cac, pisp_be_cac_extra &cac_extra, uint16_t width, uint16_t height)
{
	static const int P = PISP_BE_CAC_STEP_PRECISION;

	if (cac.grid_step_x == 0)
		cac.grid_step_x = (PISP_BE_CAC_GRID_SIZE << P) / width;
	if (cac.grid_step_y == 0)
		cac.grid_step_y = (PISP_BE_CAC_GRID_SIZE << P) / height;

	ASSERT(cac.grid_step_x * (width + cac_extra.offset_x - 1) < (PISP_BE_CAC_GRID_SIZE << P));
	ASSERT(cac.grid_step_y * (height + cac_extra.offset_y - 1) < (PISP_BE_CAC_GRID_SIZE << P));
}

void finalise_resample(pisp_be_resample_config &resample, pisp_be_resample_extra &resample_extra, uint16_t width, uint16_t height)
{
	uint32_t scale_factor_h = ((width - 1) << ScalePrecision) / (resample_extra.scaled_width - 1);
	uint32_t scale_factor_v = ((height - 1) << ScalePrecision) / (resample_extra.scaled_height - 1);

	if ((scale_factor_h < UnityScale / 16 || scale_factor_h >= 16 * UnityScale) ||
	    (scale_factor_v < UnityScale / 16 || scale_factor_v >= 16 * UnityScale))
		LOG(PISPBE, Error) << "finalise_resample: Invalid scaling factors (must be < 16x down/upscale).");

	resample.scale_factor_h = scale_factor_h;
	resample.scale_factor_v = scale_factor_v;
	// If the filter coefficients are unset we should probably copy in our "default ones".
}

void finalise_downscale(pisp_be_downscale_config &downscale, pisp_be_downscale_extra &downscale_extra, uint16_t width, uint16_t height)
{
	LOG(PISPBE, Debug) << "width " << width << " scaled_width " << downscale_extra.scaled_width;
	LOG(PISPBE, Debug) << "height " << height << " scaled_height " << downscale_extra.scaled_height;

	uint32_t scale_factor_h = (width << ScalePrecision) / (downscale_extra.scaled_width);
	uint32_t scale_factor_v = (height << ScalePrecision) / (downscale_extra.scaled_height);

	if ((scale_factor_h != UnityScale && (scale_factor_h < 2 * UnityScale || scale_factor_h > 8 * UnityScale)) ||
	    (scale_factor_v != UnityScale && (scale_factor_v < 2 * UnityScale || scale_factor_v > 8 * UnityScale)))
		LOG(PISPBE, Error) << "finalise_downscale: Invalid scaling factors (must be 1x or >= 2x && <= 8x).";

	downscale.scale_factor_h = scale_factor_h;
	downscale.scale_factor_v = scale_factor_v;
	downscale.scale_recip_h = (downscale_extra.scaled_width << ScalePrecision) / (width);
	downscale.scale_recip_v = (downscale_extra.scaled_height << ScalePrecision) / (height);

	LOG(PISPBE, Debug) << "scale_factor_h " << downscale.scale_factor_h << " scale_factor_v " << downscale.scale_factor_v;
	LOG(PISPBE, Debug) << "scale_recip_h " << downscale.scale_recip_h << " scale_recip_v " << downscale.scale_recip_v;
}

void finalise_decompression(pisp_be_config const &be_config)
{
	uint32_t fmt = be_config.input_format.format, bayer_enables = be_config.global.bayer_enables;

	if (PISP_IMAGE_FORMAT_compressed(fmt) && !(bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS))
		LOG(PISPBE, Error) << "BackEnd::finalise: input compressed but decompression not enabled";

	if (!PISP_IMAGE_FORMAT_compressed(fmt) && (bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS))
		LOG(PISPBE, Error) << "BackEnd::finalise: input uncompressed but decompression enabled";

	if ((bayer_enables & PISP_BE_BAYER_ENABLE_DECOMPRESS) && !PISP_IMAGE_FORMAT_bps_8(fmt))
		LOG(PISPBE, Error) << "BackEnd::finalise: compressed input is not 8bpp";
}

void finalise_tdn(pisp_be_config &config)
{
	int tdn_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN;
	int tdn_input_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT;
	int tdn_decompress_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS;
	int tdn_compress_enabled = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_COMPRESS;
	int tdn_output_enable = config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT;
	uint32_t fmt = config.tdn_output_format.format;

	if (tdn_enabled && !tdn_output_enable)
		LOG(PISPBE, Error) << "BackEnd::finalise: TDN output not enabled when TDN enabled";

	if (PISP_IMAGE_FORMAT_compressed(fmt) && !tdn_compress_enabled)
		LOG(PISPBE, Error) << "BackEnd::finalise: TDN output compressed but compression not enabled";

	if (!PISP_IMAGE_FORMAT_compressed(fmt) && tdn_compress_enabled)
		LOG(PISPBE, Error) << "BackEnd::finalise: TDN output uncompressed but compression enabled";

	if (tdn_compress_enabled && !PISP_IMAGE_FORMAT_bps_8(fmt))
		LOG(PISPBE, Error) << "BackEnd::finalise: TDN output does not match compression mode";

	// TDN output width/height must match the input, though the format may differ.
	config.tdn_output_format.width = config.input_format.width;
	config.tdn_output_format.height = config.input_format.height;
	compute_stride(config.tdn_output_format);

	if (!tdn_enabled) {
		if (tdn_input_enabled)
			LOG(PISPBE, Error) << "BackEnd::finalise: TDN input enabled but TDN not enabled";
		// I suppose there is a weird (and entirely pointless) case where TDN is not enabled but TDN output is, which we allow.
	} else if (config.tdn.reset) {
		if (tdn_input_enabled)
			LOG(PISPBE, Error) << "BackEnd::finalise: TDN input enabled but TDN being reset";
	} else {
		if (!tdn_input_enabled)
			LOG(PISPBE, Error) << "BackEnd::finalise: TDN input not enabled but TDN not being reset";
		// Make the TDN input match the output if it's unset. Usually this will be the sensible thing to do.
		if (config.tdn_input_format.width == 0 && config.tdn_input_format.height == 0)
			config.tdn_input_format = config.tdn_output_format;
		if (PISP_IMAGE_FORMAT_compressed(fmt) && !tdn_decompress_enabled)
			LOG(PISPBE, Error) << "BackEnd::finalise: TDN input compressed but decompression not enabled";
		if (!PISP_IMAGE_FORMAT_compressed(fmt) && tdn_decompress_enabled)
			LOG(PISPBE, Error) << "BackEnd::finalise: TDN input uncompressed but decompression enabled";
		if (tdn_compress_enabled && !PISP_IMAGE_FORMAT_bps_8(fmt))
			LOG(PISPBE, Error) << "BackEnd::finalise: TDN output does not match compression mode";
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
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch and stitch_input should be enabled/disabled together";
	if (stitch_input_enabled && PISP_IMAGE_FORMAT_compressed(input_fmt) && !stitch_decompress_enabled)
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch output compressed but decompression not enabled";
	if (stitch_input_enabled && !PISP_IMAGE_FORMAT_compressed(input_fmt) && stitch_decompress_enabled)
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch output uncompressed but decompression enabled";
	if (stitch_output_enabled && PISP_IMAGE_FORMAT_compressed(output_fmt) && !stitch_compress_enabled)
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch output compressed but compression not enabled";
	if (stitch_output_enabled && !PISP_IMAGE_FORMAT_compressed(output_fmt) && stitch_compress_enabled)
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch output uncompressed but compression enabled";
	if (stitch_decompress_enabled && !PISP_IMAGE_FORMAT_bps_8(input_fmt))
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch input does not match compression mode";
	if (stitch_compress_enabled && !PISP_IMAGE_FORMAT_bps_8(output_fmt))
		LOG(PISPBE, Error) << "BackEnd::finalise: stitch output does not match compression mode";

	if (config.stitch_output_format.width == 0 && config.stitch_output_format.height == 0) {
		config.stitch_output_format.width = config.input_format.width;
		config.stitch_output_format.height = config.input_format.height;
		compute_stride(config.stitch_output_format);
	}

	if (config.stitch_input_format.width == 0 && config.stitch_input_format.height == 0) {
		config.stitch_input_format.width = config.input_format.width;
		config.stitch_input_format.height = config.input_format.height;
		compute_stride(config.stitch_input_format);
	}

	// Compute the motion_threshold reciprocal if it hasn't been done.
	if (config.stitch.motion_threshold_recip == 0) {
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
		LOG(PISPBE, Error) << "finalise_output: output image too small";

	if (PISP_IMAGE_FORMAT_sampling_420(config.image.format) && (config.image.height & 1))
		LOG(PISPBE, Error) << "finalise_output: 420 image height should be even";

	if ((PISP_IMAGE_FORMAT_sampling_420(config.image.format) ||
             PISP_IMAGE_FORMAT_sampling_422(config.image.format)) && (config.image.width & 1))
		LOG(PISPBE, Error) << "finalise_output: 420/422 image width should be even";

	if (PISP_IMAGE_FORMAT_wallpaper(config.image.format)) {
		if ((config.image.stride & 127) || (config.image.stride2 & 127))
			LOG(PISPBE, Error) << "finalise_output: wallpaper image stride should be at least 128-byte aligned";
	} else if ((config.image.stride & 15) || (config.image.stride2 & 15))
		LOG(PISPBE, Error) << "finalise_output: image stride should be at least 16-byte aligned";
}

void check_tiles(std::vector<pisp_tile> const &tiles, uint32_t rgb_enables, unsigned int numBranches)
{
	for (auto &tile : tiles) {
		ASSERT(tile.input_width && tile.input_height); // zero inputs shouldn't be possible

		if (tile.input_width < PISP_BACK_END_MIN_TILE_WIDTH || tile.input_height < PISP_BACK_END_MIN_TILE_HEIGHT)
			LOG(PISPBE, Error) << "Tile too small at input";

		for (unsigned int i = 0; i < numBranches; i++) {
			if ((rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(i)) == 0)
				continue;

			int width_after_crop = tile.input_width - tile.crop_x_start[i] - tile.crop_x_end[i];
			int height_after_crop = tile.input_height - tile.crop_y_start[i] - tile.crop_y_end[i];

			// A tile that gets cropped away completely can't produce output, and vice versa.
			ASSERT((width_after_crop * height_after_crop == 0) == (tile.output_width[i] * tile.output_height[i] == 0));

			// A zero-sized tile is legitimate meaning "no output", but otherwise minimum tile sizes must be respected.
			if (width_after_crop && height_after_crop) {
				if (width_after_crop < PISP_BACK_END_MIN_TILE_WIDTH || height_after_crop < PISP_BACK_END_MIN_TILE_HEIGHT)
					LOG(PISPBE, Error) << "Tile too small after crop";
			
                        	if (tile.resample_in_width[i] < PISP_BACK_END_MIN_TILE_WIDTH ||
                                    tile.resample_in_height[i] < PISP_BACK_END_MIN_TILE_HEIGHT)
					LOG(PISPBE, Error) << "Tile too small after downscale";
				if (tile.output_width[i] < PISP_BACK_END_MIN_TILE_WIDTH ||
                                    tile.output_height[i] < PISP_BACK_END_MIN_TILE_HEIGHT)
					LOG(PISPBE, Error) << "Tile too small at output";
			}
		}
	}
}

unsigned int get_pixel_alignment(uint32_t format, int byte_alignment)
{
	int alignment_pixels = byte_alignment; // for 8bpp formats

	if (PISP_IMAGE_FORMAT_bps_16(format))
		alignment_pixels = byte_alignment / 2;
	else if (PISP_IMAGE_FORMAT_bps_10(format))
		alignment_pixels = byte_alignment * 3 / 4;

	if (PISP_IMAGE_FORMAT_planar(format) && !PISP_IMAGE_FORMAT_sampling_444(format))
		alignment_pixels *= 2; // the UV planes in fully planar 420/422 output will have half the width

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
	if (config.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) {
		LOG(PISPBE, Debug) << "RGB input enabled";
		// Need 4 byte alignment AND even number of pixels. Height must be 2 row aligned only for 420 input.
		return tiling::Length2(lcm(get_pixel_alignment(config.input_format.format, PISP_BACK_END_INPUT_ALIGN), 2),
			       PISP_IMAGE_FORMAT_sampling_420(config.input_format.format) ? 2 : 1);
	}

	uint32_t bayer_enables = config.global.bayer_enables;
	// For starters, we need 4 *byte* alignment (this automatically cover 2 *pixel* alignment for all the raw formats).
	int pixel_alignment = get_pixel_alignment(config.input_format.format, PISP_BACK_END_INPUT_ALIGN);

	// If any input is compressed, we need 8 *pixel* alignment.
	if (PISP_IMAGE_FORMAT_compressed(config.input_format.format) ||
	    ((bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT) && PISP_IMAGE_FORMAT_compressed(config.tdn_input_format.format)) ||
	    ((bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_INPUT) && PISP_IMAGE_FORMAT_compressed(config.stitch_input_format.format)))
		pixel_alignment = lcm(pixel_alignment, PISP_BACK_END_COMPRESSED_ALIGN);

	// If any of the Bayer outputs are enabled, those need 16 *byte* alignment. (This already covers the outputs being compressed.)
	if (bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT)
		pixel_alignment = lcm(pixel_alignment, get_pixel_alignment(config.tdn_output_format.format, PISP_BACK_END_OUTPUT_MIN_ALIGN));
	if (bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)
		pixel_alignment = lcm(pixel_alignment, get_pixel_alignment(config.stitch_output_format.format, PISP_BACK_END_OUTPUT_MIN_ALIGN));

	return tiling::Length2(pixel_alignment, 2); // Bayer input rows always in pairs
}

static tiling::Length2 calculate_output_alignment(uint32_t format, int align = PISP_BACK_END_OUTPUT_MAX_ALIGN)
{
	int y_alignment = PISP_IMAGE_FORMAT_sampling_420(format) ? 2 : 1;

	return tiling::Length2(get_pixel_alignment(format, align), y_alignment);
}

void calculate_input_addr_offset(int x, int y, pisp_image_format_config const &input_format, uint32_t *addr_offset, uint32_t *addr_offset2 = nullptr)
{
	uint32_t offset2 = 0;
	
        compute_addr_offset(input_format, x, y, addr_offset, &offset2);
	if (addr_offset2)
		*addr_offset2 = offset2;
}

} // namespace

void BackEnd::finaliseConfig()
{
	uint32_t dirty_flags_bayer = be_config_.dirty_flags_bayer & be_config_.global.bayer_enables; // only finalise blocks that are dirty *and* enabled
	uint32_t dirty_flags_rgb = be_config_.dirty_flags_rgb & be_config_.global.rgb_enables; // only finalise blocks that are dirty *and* enabled
	if ((dirty_flags_bayer & PISP_BE_BAYER_ENABLE_INPUT) || (dirty_flags_rgb & PISP_BE_RGB_ENABLE_INPUT))
		finalise_bayer_rgb_inputs(be_config_.input_format);
	if (dirty_flags_bayer & PISP_BE_BAYER_ENABLE_INPUT)
		finalise_inputs(be_config_);
	if (dirty_flags_bayer & (PISP_BE_BAYER_ENABLE_INPUT | PISP_BE_BAYER_ENABLE_DECOMPRESS))
		finalise_decompression(be_config_);
	if ((be_config_.dirty_flags_bayer &
	     (PISP_BE_BAYER_ENABLE_TDN | PISP_BE_BAYER_ENABLE_TDN_INPUT | PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS |
	      PISP_BE_BAYER_ENABLE_TDN_COMPRESS | PISP_BE_BAYER_ENABLE_TDN_OUTPUT))) {
		finalise_tdn(be_config_);
		// The address will have to be filled in by the HAL, but it needs the index. Hackily, we pass this in the address field.
		be_config_.tdn_input_buffer.addr[0] = tdn_input_index_;
		be_config_.tdn_output_buffer.addr[0] = tdn_output_index_;
		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT)
			hal_->GetTdnBuffer(tdn_input_index_).format = be_config_.tdn_input_format;
		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT)
			hal_->GetTdnBuffer(tdn_output_index_).format = be_config_.tdn_output_format;
	}
	if (be_config_.dirty_flags_bayer &
	    (PISP_BE_BAYER_ENABLE_STITCH | PISP_BE_BAYER_ENABLE_STITCH_INPUT | PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS |
	     PISP_BE_BAYER_ENABLE_STITCH_COMPRESS | PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)) {
		finalise_stitch(be_config_);
		// The address will have to be filled in by the HAL, but it needs the index. Hackily, we pass this in the address field.
		be_config_.stitch_input_buffer.addr[0] = stitch_input_index_;
		be_config_.stitch_output_buffer.addr[0] = stitch_output_index_;
		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_INPUT)
			hal_->GetStitchBuffer(stitch_input_index_).format = be_config_.stitch_input_format;
		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)
			hal_->GetStitchBuffer(stitch_output_index_).format = be_config_.stitch_output_format;
	}
	if (dirty_flags_bayer & PISP_BE_BAYER_ENABLE_LSC)
		finalise_lsc(be_config_.lsc, be_config_.lsc_extra, be_config_.input_format.width, be_config_.input_format.height);
	if (dirty_flags_bayer & PISP_BE_BAYER_ENABLE_CAC)
		finalise_cac(be_config_.cac, be_config_.cac_extra, be_config_.input_format.width, be_config_.input_format.height);
	for (int j = 0; j < NUM_OUTPUT_BRANCHES; j++) {
		bool enabled = be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(j);
		if (j == PISP_BACK_END_HOG_OUTPUT)
			enabled |= be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_HOG;
		if (enabled) {
			// crop is enabled when it contains non-zero width/height
			uint16_t w = be_config_.crop.width ? be_config_.crop.width : be_config_.input_format.width;
			uint16_t h = be_config_.crop.width ? be_config_.crop.height : be_config_.input_format.height;
			if (dirty_flags_rgb & PISP_BE_RGB_ENABLE_DOWNSCALE(j)) {
				if (pisp_variant_downscaler_available(j))
					finalise_downscale(be_config_.downscale[j], be_config_.downscale_extra[j], w, h);
				else
					LOG(PISPBE, Error) << "Downscale is not available in output branch " + std::to_string(j);
			}
			if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(j)) {
				// If the downscale is enabled, we update the input width/height for the resample stage.
				w = be_config_.downscale_extra[j].scaled_width;
				h = be_config_.downscale_extra[j].scaled_height;
			}
			if (dirty_flags_rgb & PISP_BE_RGB_ENABLE_RESAMPLE(j))
				finalise_resample(be_config_.resample[j], be_config_.resample_extra[j], w, h);

			if (dirty_flags_rgb & PISP_BE_RGB_ENABLE_OUTPUT(j))
				finalise_output(be_config_.output_format[j]);
		}
	}
	// Finally check for a sane collection of enable bits.
	if (!((be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) || (be_config_.global.bayer_enables == 0)))
		LOG(PISPBE, Error) << "BackEnd::finalise: Bayer input disabled but Bayer pipe active";
	if (!!(be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) + !!(be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) != 1)
		LOG(PISPBE, Error) << "BackEnd::finalise: exactly one of Bayer and RGB inputs should be enabled";
	uint32_t output_enables = be_config_.global.bayer_enables & (PISP_BE_BAYER_ENABLE_TDN_OUTPUT | PISP_BE_BAYER_ENABLE_STITCH_OUTPUT);
	for (int i = 0; i < variant_.numBackEndBranches(0); i++)
		output_enables |= be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(i);
	output_enables |= be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_HOG;
	if (output_enables == 0)
		LOG(PISPBE, Error) << "BackEnd::finalise: PiSP not configured to do anything";
}

void BackEnd::updateTiles()
{
	static_ASSERT(NUM_OUTPUT_BRANCHES == variant_.numBackEndBranches(0),
		      "NUM_OUTPUT_BRANCHES and variant_.numBackEndBranches(0) should be the same");
	if (retile_) {
		retile_ = false;
		TilingConfig tiling_config;
		pisp_be_config const &c = be_config_;
		tiling_config.input_alignment = calculate_input_alignment(c);
		LOG(PISPBE, Debug) << "Input alignments are " << tiling_config.input_alignment << " pixels";
		tiling_config.input_image_size = tiling::Length2(c.input_format.width, c.input_format.height);
		tiling_config.crop = Interval2(Interval(c.crop.offset_x, c.crop.width),
					       Interval(c.crop.offset_y, c.crop.height));
		if (tiling_config.crop.x.length == 0 || tiling_config.crop.y.length == 0)
			tiling_config.crop =
				Interval2(Interval(0, c.input_format.width), Interval(0, c.input_format.height));
		for (int i = 0; i < NUM_OUTPUT_BRANCHES; i++) {
			tiling_config.output_h_mirror[i] = be_config_.output_format[i].transform & PISP_BE_TRANSFORM_HFLIP;
			tiling_config.downscale_factor[i] = tiling::Length2(c.downscale[i].scale_factor_h, c.downscale[i].scale_factor_v);
			tiling_config.resample_factor[i] = tiling::Length2(c.resample[i].scale_factor_h, c.resample[i].scale_factor_v);
			tiling_config.downscale_image_size[i] = tiling::Length2(c.downscale_extra[i].scaled_width, c.downscale_extra[i].scaled_height);
			tiling_config.output_image_size[i] = tiling::Length2(c.output_format[i].image.width, c.output_format[i].image.height);
			tiling_config.output_max_alignment[i] = calculate_output_alignment(c.output_format[i].image.format, PISP_BACK_END_OUTPUT_MAX_ALIGN);
			tiling_config.output_min_alignment[i] = calculate_output_alignment(c.output_format[i].image.format, PISP_BACK_END_OUTPUT_MIN_ALIGN);
		}
		// If HOG output is enabled, but the corresponding regular output isn't, we'll have to for that branch to get tiled up too.
		if ((c.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(PISP_BACK_END_HOG_OUTPUT)) == 0 &&
		    (c.global.rgb_enables & PISP_BE_RGB_ENABLE_HOG)) {
			uint16_t width, height;
			getOutputSize(PISP_BACK_END_HOG_OUTPUT, &width, &height, be_config_.input_format);
			tiling_config.output_image_size[PISP_BACK_END_HOG_OUTPUT] = tiling::Length2(width, height);
			tiling_config.output_min_alignment[PISP_BACK_END_HOG_OUTPUT] = tiling::Length2(8, 1); // I think 8 is basically right
			tiling_config.output_max_alignment[PISP_BACK_END_HOG_OUTPUT] = tiling::Length2(32, 1); // and this one probably doesn't much matter
		}
		tiling_config.max_tile_size.dx = config_.max_tile_width ? config_.max_tile_width : pisp_variant_get_back_end_max_tile_width(GetId());
		tiling_config.max_tile_size.dy = config_.max_stripe_height ? config_.max_stripe_height : MaxStripeHeight;
		tiling_config.min_tile_size = tiling::Length2(PISP_BACK_END_MIN_TILE_WIDTH, PISP_BACK_END_MIN_TILE_HEIGHT);
		tiling_config.resample_enables = be_config_.global.rgb_enables / (int)PISP_BE_RGB_ENABLE_RESAMPLE0;
		tiling_config.downscale_enables = be_config_.global.rgb_enables / (int)PISP_BE_RGB_ENABLE_DOWNSCALE0;
		// Set compressed_input to false as otherwise the tiling would pad tiles up to multiples of 8 pixels even when these lie
		// outside the actual image width (and we've chosen not to handle compression like that).
		tiling_config.compressed_input = false;
		tiles_ = retilePipeline(tiling_config);
		check_tiles(tiles_, c.global.rgb_enables, variant_.numBackEndBranches(0));
		finalise_tiling_ = true;
	}
	if (finalise_tiling_) {
		finaliseTiling(tiles_);
		finalise_tiling_ = false;
	}
}

std::vector<PISP_TILE_T> BackEnd::retilePipeline(TilingConfig const &tiling_config)
{
	// The tiling library provides tiles in a SW Tile structure.
	Tile tiles[PISP_BACK_END_NUM_TILES];
	Length2 grid;
	tile_pipeline(tiling_config, tiles, PISP_BACK_END_NUM_TILES, &grid);
	num_tiles_x_ = grid.dx;
	num_tiles_y_ = grid.dy;
	std::vector<PISP_TILE_T> tile_vector(num_tiles_x_ * num_tiles_y_);
	// Finally convert the Tiles into PISP_TILE_Ts.
	for (int i = 0; i < num_tiles_x_ * num_tiles_y_; i++) {
		PISP_TILE_T &t = tile_vector[i];
		memset(&t, 0, sizeof(PISP_TILE_T));
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
			LOG(PISPBE, Error) << "BackEnd::retilePipeline: tiling error in Bayer pipe";

		for (int j = 0; j < NUM_OUTPUT_BRANCHES; j++) {
			bool enabled = (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT(j));
			if (j == PISP_BACK_END_HOG_OUTPUT)
				enabled |= (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_HOG);
			if (enabled && (tiles[i].output[j].output.x.length == 0 || tiles[i].output[j].output.y.length == 0)) {
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

			Crop2 downscale_crop;
			Interval2 resample_size = tiles[i].crop.output;
			resample_size.x = resample_size.x - tiles[i].resample[j].crop.x;
			resample_size.y = resample_size.y - tiles[i].resample[j].crop.y;
			// When a resize stage is disabled, the tile size after the stage is found from the input of the
			// next block. Also there will be no extra crop necessary for the resize operation.
			if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(j)) {
				downscale_crop = tiles[i].downscale[j].crop + tiles[i].crop.crop;
				// Size of the tile going into the resample block needs to be set here.
				resample_size = tiles[i].downscale[j].output;
			} else if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(j))
				downscale_crop = tiles[i].resample[j].crop + tiles[i].crop.crop;
			else
				downscale_crop = tiles[i].output[j].crop + tiles[i].crop.crop;

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

			for (int p = 0; p < 3; p++) {
				// Calculate x/y initial downsampler/resampler phases per-plane.
				if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_DOWNSCALE(j)) {
					unsigned int frac_x = (resample_size.x.offset * be_config_.downscale[j].scale_factor_h) & ((1 << ScalePrecision) - 1);
					unsigned int frac_y = (resample_size.y.offset * be_config_.downscale[j].scale_factor_v) & ((1 << ScalePrecision) - 1);
					// Fractional component of the input required to generate the output pixel.
					t.downscale_phase_x[p * NUM_OUTPUT_BRANCHES + j] = (UnityScale acx ); 
				t.downscale_phase_y[p * NUM_OUTPUT_BRANCHES + j] = (UnityScale acy ); 
			}
				if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(j)) {
					const int NUM_PHASES = 16;
					// Location of the output pixel in the interpolated (input) image.
					unsigned int interpolated_pix_x = (t.output_offset_x[j] * NUM_PHASES * be_config_.resample[j].scale_factor_h) >> ScalePrecision;
					unsigned int interpolated_pix_y = (t.output_offset_y[j] * NUM_PHASES * be_config_.resample[j].scale_factor_v) >> ScalePrecision;
					// Phase of the interpolated input pixel.
					t.resample_phase_x[p * NUM_OUTPUT_BRANCHES + j] = ((interpolated_pix_x % NUM_PHASES) << ScalePrecision) / NUM_PHASES;
					t.resample_phase_y[p * NUM_OUTPUT_BRANCHES + j] = ((interpolated_pix_y % NUM_PHASES) << ScalePrecision) / NUM_PHASES;
					// Account for any user defined initial phase - this could be negative!
					t.resample_phase_x[p * NUM_OUTPUT_BRANCHES + j] += be_config_.resample_extra[j].initial_phase_h[p];
					t.resample_phase_y[p * NUM_OUTPUT_BRANCHES + j] += be_config_.resample_extra[j].initial_phase_v[p];
					// Have to be within this range, else some calculation went wrong.
					ASSERT(t.resample_phase_x[p * NUM_OUTPUT_BRANCHES + j] <= 2 * (UnityScale );	 		 		SSERT(t.resample_phase_y[p * NUM_OUTPUT_BRANCHES + j] <= 2 * (UnityScale );	 		 	}			}

			// Phase difference between planes cannot be > 0.5 pixels on the output dimenstions.
			if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_RESAMPLE(j)) {
				int phase_max = (be_config_.resample[j].scale_factor_h * UnityScale  >  Sc alPrecision;
				if (std::abs(t.resample_phase_x[0 * NUM_OUTPUT_BRANCHES + j] - t.resample_phase_x[1 * NUM_OUTPUT_BRANCHES + j]) > phase_max ||
				    std::abs(t.resample_phase_x[1 * NUM_OUTPUT_BRANCHES + j] - t.resample_phase_x[2 * NUM_OUTPUT_BRANCHES + j]) > phase_max ||
				    std::abs(t.resample_phase_x[0 * NUM_OUTPUT_BRANCHES + j] - t.resample_phase_x[2 * NUM_OUTPUT_BRANCHES + j]) > phase_max) {
					LOG(PISPBE, Error) << "Resample phase x for tile is > 0.5 pixels on the output dimensions.";
				}
				phase_max = (be_config_.resample[j].scale_factor_v * UnityScale  >  Sc alPrecision;
				if (std::abs(t.resample_phase_y[0 * NUM_OUTPUT_BRANCHES + j] - t.resample_phase_y[1 * NUM_OUTPUT_BRANCHES + j]) > phase_max ||
				    std::abs(t.resample_phase_y[1 * NUM_OUTPUT_BRANCHES + j] - t.resample_phase_y[2 * NUM_OUTPUT_BRANCHES + j]) > phase_max ||
				    std::abs(t.resample_phase_y[0 * NUM_OUTPUT_BRANCHES + j] - t.resample_phase_y[2 * NUM_OUTPUT_BRANCHES + j]) > phase_max) {
					LOG(PISPBE, Error) << "Resample phase y for tile is > 0.5 pixels on the output dimensions.";
				}
			}
		}
	}
	return tile_vector;
}

void BackEnd::finaliseTiling(std::vector<PISP_TILE_T> &tiles)
{
	// Update tile parameters (offsets/strides) from on the BE pipeline configuration.
	for (PISP_TILE_T &t : tiles) {
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.input_format, &t.input_addr_offset, &t.input_addr_offset2);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.tdn_input_format, &t.tdn_input_addr_offset);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.tdn_output_format, &t.tdn_output_addr_offset);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.stitch_input_format, &t.stitch_input_addr_offset);
		calculate_input_addr_offset(t.input_offset_x, t.input_offset_y, be_config_.stitch_output_format, &t.stitch_output_addr_offset);
		LOG(PISPBE, Debug) << "Input offsets " << t.input_offset_x << "," << t.input_offset_y << " address offsets " << t.input_addr_offset << " and " << .input_addr_offset2);

		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_LSC) {
			t.lsc_grid_offset_x = (t.input_offset_x + be_config_.lsc_extra.offset_x) * be_config_.lsc.grid_step_x;
			t.lsc_grid_offset_y = (t.input_offset_y + be_config_.lsc_extra.offset_y) * be_config_.lsc.grid_step_y;
		}
		if (be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_CAC) {
			t.cac_grid_offset_x = (t.input_offset_x + be_config_.cac_extra.offset_x) * be_config_.cac.grid_step_x;
			t.cac_grid_offset_y = (t.input_offset_y + be_config_.cac_extra.offset_y) * be_config_.cac.grid_step_y;
		}

		for (int j = 0; j < NUM_OUTPUT_BRANCHES; j++) {
			int output_offset_x_unflipped = t.output_offset_x[j], output_offset_y_unflipped = t.output_offset_y[j];
			if (be_config_.output_format[j].transform & PISP_BE_TRANSFORM_HFLIP)
				t.output_offset_x[j] = be_config_.output_format[j].image.width - output_offset_x_unflipped - t.output_width[j];
			if (be_config_.output_format[j].transform & PISP_BE_TRANSFORM_VFLIP)
				t.output_offset_y[j] = be_config_.output_format[j].image.height - output_offset_y_unflipped - 1;
			PISP_IMAGE_FORMAT_compute_addr_offset(&be_config_.output_format[j].image, t.output_offset_x[j], t.output_offset_y[j],
							      &t.output_addr_offset[j], &t.output_addr_offset2[j]);
			LOG(PISPBE, Debug) << "Branch " << j << " output offsets " << t.output_offset_x[j] << "," << t.output_offset_y[j] << " address offsets " << .output_addr_offset[j] << " and " << t.output_addr_offset2[j]);

			if (j == PISP_BACK_END_HOG_OUTPUT) {
				if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_HOG) {
					// Convert image dimenstions to cell dimensions.  Remember, these are cell offsets.
					// Use *unflipped* offsets as HOG doesn't flip its output.
					int cell_offset_x = output_offset_x_unflipped / HogCellSize;
					int cell_offset_y = output_offset_y_unflipped / HogCellSize;
					PISP_IMAGE_FORMAT_compute_addr_offset(&be_config_.hog_format, cell_offset_x, cell_offset_y,
									      &t.output_hog_addr_offset, nullptr);
				}
			}
		}
	}
}

void BackEnd::Prepare()
{
	LOG(PISPBE, Debug) << "New frame!";

	bool integral_image_output = false;

	// On every start-of-frame we:
	// 1. Check the input configuration appears sensible.
	if ((be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) == 0 &&
	    (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) == 0)
		LOG(PISPBE, Error) << "BackEnd::preFrameUpdate: neither Bayer nor RGB inputs are enabled";
	else if ((be_config_.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT) &&
		 (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT))
		LOG(PISPBE, Error) << "BackEnd::preFrameUpdate: both Bayer and RGB inputs are enabled";

	// 2. Also check the output configuration (including HOG) is all filled in and looks sensible. Again, addresses must be
	// left to the HAL.
	for (unsigned int i = 0; i < variant_.numBackEndBranches(0); i++) {

		pisp_image_format_config &config = be_config_.output_format[i].image;
		ComputeOutputImageFormat(i, config, be_config_.input_format);

		if (config.format & PISP_IMAGE_FORMAT_INTEGRAL_IMAGE) {
			if (!pisp_variant_get_integral_image_support())
				LOG(PISPBE, Error) << "Integral images are not supported in the current configuration.";
			integral_image_output = true;
		}
	}

	if (be_config_.global.rgb_enables & PISP_BE_RGB_ENABLE_HOG) {
		ComputeHogOutputImageFormat(be_config_.hog_format, be_config_.input_format);
		be_config_.hog.stride = be_config_.hog_format.stride;
	}

	// 3. Fill in any other missing bits of config, and update the tiling if necessary.
	finaliseConfig();
	updateTiles();

	// Sanity check - integral images are only valid for a single tile output.
	ASSERT((num_tiles_x_ * num_tiles_y_ == 1) || !integral_image_output);
}