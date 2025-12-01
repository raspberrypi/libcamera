
/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * frontend.cpp - PiSP Front End implementation
 */
#include "frontend.hpp"

#include <cstddef>
#include <type_traits>

#include "common/logging.hpp"
#include "common/utils.hpp"

using namespace libpisp;

namespace
{

inline uint32_t block_enable(uint32_t block, unsigned int branch)
{
	return block << (4 * branch);
}

void finalise_lsc(pisp_fe_lsc_config &lsc, uint16_t width, uint16_t height)
{
	if (lsc.centre_x == 0)
		lsc.centre_x = width / 2;

	if (lsc.centre_y == 0)
		lsc.centre_y = height / 2;

	if (lsc.scale == 0)
	{
		uint16_t max_dx = std::max<int>(width - lsc.centre_x, lsc.centre_x);
		uint16_t max_dy = std::max<int>(height - lsc.centre_y, lsc.centre_y);
		uint32_t max_r2 = max_dx * (uint32_t)max_dx + max_dy * (uint32_t)max_dy;

		// spec requires r^2 to fit 31 bits
		PISP_ASSERT(max_r2 < (1u << 31));

		lsc.shift = 0;
		while (max_r2 >= 2 * ((PISP_FE_LSC_LUT_SIZE - 1) << FrontEnd::InterpPrecision))
		{
			max_r2 >>= 1;
			lsc.shift++;
		}

		lsc.scale =
			((1 << FrontEnd::ScalePrecision) * ((PISP_FE_LSC_LUT_SIZE - 1) << FrontEnd::InterpPrecision) - 1) / max_r2;
		if (lsc.scale >= (1 << FrontEnd::ScalePrecision))
			lsc.scale = (1 << FrontEnd::ScalePrecision) - 1;
	}
}

void finalise_agc(pisp_fe_agc_stats_config &agc, uint16_t width, uint16_t height)
{
	if (agc.size_x == 0)
		agc.size_x = std::max(2, ((width - 2 * agc.offset_x) / PISP_AGC_STATS_SIZE) & ~1);
	if (agc.size_y == 0)
		agc.size_y = std::max(2, ((height - 2 * agc.offset_y) / PISP_AGC_STATS_SIZE) & ~1);
	if (agc.row_size_x == 0)
		agc.row_size_x = std::max(2, (width - 2 * agc.row_offset_x) & ~1);
	if (agc.row_size_y == 0)
		agc.row_size_y = std::max(2, ((height - 2 * agc.row_offset_y) / PISP_AGC_STATS_NUM_ROW_SUMS) & ~1);
}

void finalise_awb(pisp_fe_awb_stats_config &awb, uint16_t width, uint16_t height)
{
	 // Just a warning that ACLS algorithms might want the size calculations
	 // here to match the Back End LSC. Here we round the cell width and height
	// to the nearest even number.
	if (awb.size_x == 0)
		awb.size_x = 2 * std::max(1, ((width - 2 * awb.offset_x + PISP_AWB_STATS_SIZE) / (2 * PISP_AWB_STATS_SIZE)));

	if (awb.size_y == 0)
		awb.size_y = 2 * std::max(1, ((height - 2 * awb.offset_y + PISP_AWB_STATS_SIZE) / (2 * PISP_AWB_STATS_SIZE)));
}

void finalise_cdaf(pisp_fe_cdaf_stats_config &cdaf, uint16_t width, uint16_t height)
{
	if (cdaf.size_x == 0)
		cdaf.size_x = std::max(2, ((width - 2 * cdaf.offset_x) / PISP_CDAF_STATS_SIZE) & ~1);
	if (cdaf.size_y == 0)
		cdaf.size_y = std::max(2, ((height - 2 * cdaf.offset_y) / PISP_CDAF_STATS_SIZE) & ~1);
}

void finalise_downscale(pisp_fe_downscale_config &downscale, uint16_t width, uint16_t height)
{
	downscale.output_width = (((width >> 1) * downscale.xout) / downscale.xin) * 2;
	downscale.output_height = (((height >> 1) * downscale.yout) / downscale.yin) * 2;
}

void finalise_compression(pisp_fe_config const &fe_config, int i)
{
	uint32_t fmt = fe_config.ch[i].output.format.format;
	uint32_t enables = fe_config.global.enables;

	if (PISP_IMAGE_FORMAT_COMPRESSED(fmt) && !(enables & block_enable(PISP_FE_ENABLE_COMPRESS0, i)))
		PISP_LOG(fatal, "FrontEnd::finalise: output compressed but compression not enabled");

	if (!PISP_IMAGE_FORMAT_COMPRESSED(fmt) && (enables & block_enable(PISP_FE_ENABLE_COMPRESS0, i)))
		PISP_LOG(fatal, "FrontEnd::finalise: output uncompressed but compression enabled");

	if ((enables & block_enable(PISP_FE_ENABLE_COMPRESS0, i)) && !PISP_IMAGE_FORMAT_BPS_8(fmt))
		PISP_LOG(fatal, "FrontEnd::finalise: compressed output is not 8 bit");
}

template <typename T>
std::enable_if_t<std::is_integral_v<T>> inline div2_round_e(T &val)
{
	// Divide by 2 and round to the next even number.
	val = ((val + 2) & ~3) >> 1;
}

void decimate_config(pisp_fe_config &fe_config)
{
	if (fe_config.global.enables & PISP_FE_ENABLE_LSC)
	{
		div2_round_e(fe_config.lsc.centre_x);
		div2_round_e(fe_config.lsc.centre_y);
	}

	if (fe_config.global.enables & PISP_FE_ENABLE_CDAF_STATS)
	{
		div2_round_e(fe_config.cdaf_stats.offset_x);
		div2_round_e(fe_config.cdaf_stats.offset_y);
		div2_round_e(fe_config.cdaf_stats.size_x);
		div2_round_e(fe_config.cdaf_stats.size_y);
		div2_round_e(fe_config.cdaf_stats.skip_x);
		div2_round_e(fe_config.cdaf_stats.skip_y);
	}

	if (fe_config.global.enables & PISP_FE_ENABLE_AWB_STATS)
	{
		div2_round_e(fe_config.awb_stats.offset_x);
		div2_round_e(fe_config.awb_stats.offset_y);
		div2_round_e(fe_config.awb_stats.size_x);
		div2_round_e(fe_config.awb_stats.size_y);
	}

	if (fe_config.global.enables & PISP_FE_ENABLE_AGC_STATS)
	{
		div2_round_e(fe_config.agc_stats.offset_x);
		div2_round_e(fe_config.agc_stats.offset_y);
		div2_round_e(fe_config.agc_stats.size_x);
		div2_round_e(fe_config.agc_stats.size_y);
		div2_round_e(fe_config.agc_stats.row_offset_x);
		div2_round_e(fe_config.agc_stats.row_offset_y);
		div2_round_e(fe_config.agc_stats.row_size_x);
		div2_round_e(fe_config.agc_stats.row_size_y);
	}

	for (unsigned int i = 0; i < PISP_FLOATING_STATS_NUM_ZONES; i++)
	{
		pisp_fe_floating_stats_region &region = fe_config.floating_stats.regions[i];
		div2_round_e(region.offset_x);
		div2_round_e(region.offset_y);
		div2_round_e(region.size_x);
		div2_round_e(region.size_y);
	}
}

} // namespace

FrontEnd::FrontEnd(bool streaming, PiSPVariant const &variant, int align) : variant_(variant), align_(align)
{
	pisp_fe_input_config input;

	memset(&fe_config_, 0, sizeof(fe_config_));
	memset(&input, 0, sizeof(input));

	input.streaming = !!streaming;

	// Configure some plausible default AXI reader settings.
	if (!input.streaming)
	{
		input.axi.maxlen_flags = PISP_AXI_FLAG_ALIGN | 7;
		input.axi.cache_prot = 0x33;
		input.axi.qos = 0;
		input.holdoff = 0;
	}
	else
	{
		fe_config_.output_axi.maxlen_flags = 0xaf;
		fe_config_.output_axi.cache_prot = 0x32;
		fe_config_.output_axi.qos = 0x8410;
		fe_config_.output_axi.thresh = 0x0140;
		fe_config_.output_axi.throttle = 0x4100;
		fe_config_.dirty_flags_extra |= PISP_FE_DIRTY_OUTPUT_AXI;
	}

	pisp_fe_global_config global;
	GetGlobal(global);
	global.enables |= PISP_FE_ENABLE_INPUT;
	SetGlobal(global);
	SetInput(input);
}

FrontEnd::~FrontEnd()
{
}

void FrontEnd::SetGlobal(pisp_fe_global_config const &global)
{
	// label anything that has become enabled as dirty
	fe_config_.dirty_flags |= (global.enables & ~fe_config_.global.enables);
	fe_config_.global = global;
	fe_config_.dirty_flags_extra |= PISP_FE_DIRTY_GLOBAL;
}

void FrontEnd::GetGlobal(pisp_fe_global_config &global) const
{
	global = fe_config_.global;
}

void FrontEnd::SetInput(pisp_fe_input_config const &input)
{
	fe_config_.input = input;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_INPUT;
}

void FrontEnd::SetDecompress(pisp_decompress_config const &decompress)
{
	fe_config_.decompress = decompress;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DECOMPRESS;
}

void FrontEnd::SetDecompand(pisp_fe_decompand_config const &decompand)
{
	fe_config_.decompand = decompand;
	fe_config_.decompand.pad = 0;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DECOMPAND;
}

void FrontEnd::SetDpc(pisp_fe_dpc_config const &dpc)
{
	fe_config_.dpc = dpc;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_DPC;
}

void FrontEnd::SetBla(pisp_bla_config const &bla)
{
	fe_config_.bla = bla;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_BLA;
}

void FrontEnd::SetStatsCrop(pisp_fe_crop_config const &stats_crop)
{
	fe_config_.stats_crop = stats_crop;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_STATS_CROP;
}

void FrontEnd::SetBlc(pisp_bla_config const &blc)
{
	fe_config_.blc = blc;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_BLC;
}

void FrontEnd::SetLsc(pisp_fe_lsc_config const &lsc)
{
	fe_config_.lsc = lsc;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_LSC;
}

void FrontEnd::SetRGBY(pisp_fe_rgby_config const &rgby)
{
	fe_config_.rgby = rgby;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_RGBY;
}

void FrontEnd::SetAgcStats(pisp_fe_agc_stats_config const &agc_stats)
{
	fe_config_.agc_stats = agc_stats;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_AGC_STATS;
}

void FrontEnd::GetAgcStats(pisp_fe_agc_stats_config &agc_stats) const
{
	agc_stats = fe_config_.agc_stats;
}

void FrontEnd::SetAwbStats(pisp_fe_awb_stats_config const &awb_stats)
{
	fe_config_.awb_stats = awb_stats;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_AWB_STATS;
}

void FrontEnd::GetAwbStats(pisp_fe_awb_stats_config &awb_stats) const
{
	awb_stats = fe_config_.awb_stats;
}

void FrontEnd::SetFloatingStats(pisp_fe_floating_stats_config const &floating_stats)
{
	fe_config_.floating_stats = floating_stats;
	fe_config_.dirty_flags_extra |= PISP_FE_DIRTY_FLOATING;
}

void FrontEnd::SetCdafStats(pisp_fe_cdaf_stats_config const &cdaf_stats)
{
	fe_config_.cdaf_stats = cdaf_stats;
	fe_config_.dirty_flags |= PISP_FE_ENABLE_CDAF_STATS;
}

void FrontEnd::GetCdafStats(pisp_fe_cdaf_stats_config &cdaf_stats) const
{
	cdaf_stats = fe_config_.cdaf_stats;
}

void FrontEnd::SetCrop(unsigned int output_num, pisp_fe_crop_config const &crop)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));

	fe_config_.ch[output_num].crop = crop;
	fe_config_.dirty_flags |= block_enable(PISP_FE_ENABLE_CROP0, output_num);
}

void FrontEnd::SetDownscale(unsigned int output_num, pisp_fe_downscale_config const &downscale)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	PISP_ASSERT(variant_.FrontEndDownscalerAvailable(0, output_num));

	fe_config_.ch[output_num].downscale = downscale;
	fe_config_.dirty_flags |= block_enable(PISP_FE_ENABLE_DOWNSCALE0, output_num);
}

void FrontEnd::SetCompress(unsigned int output_num, pisp_compress_config const &compress)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));

	fe_config_.ch[output_num].compress = compress;
	fe_config_.dirty_flags |= block_enable(PISP_FE_ENABLE_COMPRESS0, output_num);
}

void FrontEnd::SetOutputFormat(unsigned int output_num, pisp_image_format_config const &output_format)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));

	fe_config_.ch[output_num].output.format = output_format;
	fe_config_.dirty_flags |= block_enable(PISP_FE_ENABLE_OUTPUT0, output_num);
}

void FrontEnd::SetOutputIntrLines(unsigned int output_num, int ilines)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));

	fe_config_.ch[output_num].output.ilines = ilines;
	fe_config_.dirty_flags |= block_enable(PISP_FE_ENABLE_OUTPUT0, output_num);
}

void FrontEnd::SetOutputBuffer(unsigned int output_num, pisp_fe_output_buffer_config const &output_buffer)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));

	fe_config_.output_buffer[output_num] = output_buffer;
	// Assume these always get written.
}

void FrontEnd::GetInput(pisp_fe_input_config &input) const
{
	input = fe_config_.input;
}

void FrontEnd::GetInputBuffer(pisp_fe_input_buffer_config &input_buffer) const
{
	input_buffer = fe_config_.input_buffer;
}

void FrontEnd::GetDecompress(pisp_decompress_config &decompress) const
{
	decompress = fe_config_.decompress;
}

void FrontEnd::GetDecompand(pisp_fe_decompand_config &decompand) const
{
	decompand = fe_config_.decompand;
}

void FrontEnd::GetDpc(pisp_fe_dpc_config &dpc) const
{
	dpc = fe_config_.dpc;
}

void FrontEnd::GetBla(pisp_bla_config &bla) const
{
	bla = fe_config_.bla;
}

void FrontEnd::GetStatsCrop(pisp_fe_crop_config &stats_crop) const
{
	stats_crop = fe_config_.stats_crop;
}

void FrontEnd::GetBlc(pisp_bla_config &blc) const
{
	blc = fe_config_.blc;
}

void FrontEnd::GetRGBY(pisp_fe_rgby_config &rgby) const
{
	rgby = fe_config_.rgby;
}

void FrontEnd::GetLsc(pisp_fe_lsc_config &lsc) const
{
	lsc = fe_config_.lsc;
}

void FrontEnd::GetFloatingStats(pisp_fe_floating_stats_config &floating_stats) const
{
	floating_stats = fe_config_.floating_stats;
}

void FrontEnd::GetCrop(unsigned int output_num, pisp_fe_crop_config &crop) const
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	crop = fe_config_.ch[output_num].crop;
}

void FrontEnd::GetDownscale(unsigned int output_num, pisp_fe_downscale_config &downscale) const
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	downscale = fe_config_.ch[output_num].downscale;
}

void FrontEnd::GetCompress(unsigned int output_num, pisp_compress_config &compress) const
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	compress = fe_config_.ch[output_num].compress;
}

void FrontEnd::GetOutputFormat(unsigned int output_num, pisp_image_format_config &output_format) const
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	output_format = fe_config_.ch[output_num].output.format;
}

void FrontEnd::GetOutputBuffer(unsigned int output_num, pisp_fe_output_buffer_config &output_buffer) const
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	output_buffer = fe_config_.output_buffer[output_num];
}

int FrontEnd::GetOutputIntrLines(unsigned int output_num) const
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));
	return fe_config_.ch[output_num].output.ilines;
}

void FrontEnd::Prepare(pisp_fe_config *config)
{
	// Only finalise blocks that are dirty *and* enabled.
	uint32_t dirty_flags = fe_config_.dirty_flags & fe_config_.global.enables;
	uint16_t width = fe_config_.input.format.width, height = fe_config_.input.format.height;

	if (fe_config_.global.enables & PISP_FE_ENABLE_STATS_CROP)
	{
		width = fe_config_.stats_crop.width;
		height = fe_config_.stats_crop.height;
	}

	if (dirty_flags & PISP_FE_ENABLE_LSC)
		finalise_lsc(fe_config_.lsc, width, height);
	if (dirty_flags & PISP_FE_ENABLE_AGC_STATS)
		finalise_agc(fe_config_.agc_stats, width, height);
	if (dirty_flags & PISP_FE_ENABLE_AWB_STATS)
		finalise_awb(fe_config_.awb_stats, width, height);
	if (dirty_flags & PISP_FE_ENABLE_CDAF_STATS)
		finalise_cdaf(fe_config_.cdaf_stats, width, height);

	width = fe_config_.input.format.width, height = fe_config_.input.format.height;
	for (int i = 0; i < PISP_FE_NUM_OUTPUTS; i++)
	{
		if (dirty_flags & block_enable(PISP_FE_ENABLE_DOWNSCALE0, i))
		{
			int cwidth = width, cheight = height;

			if (fe_config_.global.enables & block_enable(PISP_FE_ENABLE_CROP0, i))
				cwidth = fe_config_.ch[i].crop.width, cheight = fe_config_.ch[i].crop.height;

			finalise_downscale(fe_config_.ch[i].downscale, cwidth, cheight);
		}

		if (dirty_flags & (block_enable(PISP_FE_ENABLE_OUTPUT0, i) | block_enable(PISP_FE_ENABLE_COMPRESS0, i)))
			finalise_compression(fe_config_, i);

		if (dirty_flags & block_enable(PISP_FE_ENABLE_OUTPUT0, i))
		{
			pisp_image_format_config &image_config = fe_config_.ch[i].output.format;

			fixOutputSize(i);
			if (!image_config.stride)
				compute_stride_align(image_config, align_);
		}
	}

	*config = fe_config_;

	// Fixup any grid offsets/sizes if stats decimation is enabled.
	if (config->global.enables & PISP_FE_ENABLE_DECIMATE)
		decimate_config(*config);

	fe_config_.dirty_flags = fe_config_.dirty_flags_extra = 0;
}

void FrontEnd::fixOutputSize(unsigned int output_num)
{
	PISP_ASSERT(output_num < variant_.FrontEndNumBranches(0));

	pisp_image_format_config &image_config = fe_config_.ch[output_num].output.format;

	image_config.width = image_config.height = 0;

	if (fe_config_.global.enables & block_enable(PISP_FE_ENABLE_OUTPUT0, output_num))
	{
		image_config.width = fe_config_.input.format.width;
		image_config.height = fe_config_.input.format.height;

		if (fe_config_.global.enables & block_enable(PISP_FE_ENABLE_CROP0, output_num))
		{
			image_config.width = fe_config_.ch[output_num].crop.width;
			image_config.width = fe_config_.ch[output_num].crop.height;
		}

		if (fe_config_.global.enables & block_enable(PISP_FE_ENABLE_DOWNSCALE0, output_num))
		{
			image_config.width = fe_config_.ch[output_num].downscale.output_width;
			image_config.height = fe_config_.ch[output_num].downscale.output_height;
		}
	}
}
