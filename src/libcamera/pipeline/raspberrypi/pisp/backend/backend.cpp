// Implementation of the PiSP Back End driver.
#include "backend.h"

#include <libcamera/base/log.h>

using namespace libcamera;
using namespace PiSP;

LOG_DEFINE_CATEGORY(PISPBE)

BackEnd::BackEnd(Config const &config, PiSPVariant const &variant)
	: config_(config), variant_(variant), retile_(true), finalise_tiling_(true)
{
	unsigned int max_tile_width = variant_.backEndMaxTileWidth(0);

        if (config_.max_tile_width > max_tile_width)
		LOG(PISPBE, Error) << "Configured max tile width " << config_.max_tile_width << " exceeds " << max_tile_width;

	InitialiseConfig();
}

BackEnd::~BackEnd()
{
}

void BackEnd::SetGlobal(pisp_be_global_config const &global)
{
	uint32_t changed_rgb_enables = (global.rgb_enables ^ be_config_.global.rgb_enables);

	if (changed_rgb_enables &
	    (PISP_BE_RGB_ENABLE_DOWNSCALE0 | PISP_BE_RGB_ENABLE_DOWNSCALE1 | PISP_BE_RGB_ENABLE_RESAMPLE0 | PISP_BE_RGB_ENABLE_RESAMPLE1 | PISP_BE_RGB_ENABLE_HOG))
		retile_ = true; // must retile when rescaling OR HoG blocks change

	be_config_.dirty_flags_bayer |= (global.bayer_enables & ~be_config_.global.bayer_enables); // label anything newly enabled as dirty
	be_config_.dirty_flags_rgb |= (global.rgb_enables & ~be_config_.global.rgb_enables); // label anything newly enabled as dirty
	be_config_.global = global;
	be_config_.global.pad[0] = be_config_.global.pad[1] = be_config_.global.pad[2] = 0;
	be_config_.dirty_flags_extra |= PISP_BE_DIRTY_GLOBAL;
}

void BackEnd::GetGlobal(pisp_be_global_config &global) const
{
	global = be_config_.global;
}

void BackEnd::SetInputFormat(pisp_image_format_config const &input_format)
{
	be_config_.input_format = input_format;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_INPUT;
	retile_ = true;
}

void BackEnd::SetInputBuffer(pisp_be_input_buffer_config const &input_buffer)
{
	be_config_.input_buffer = input_buffer;
}

void BackEnd::SetDecompress(pisp_decompress_config const &decompress)
{
	be_config_.decompress = decompress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DECOMPRESS;
}

void BackEnd::SetDpc(pisp_be_dpc_config const &dpc)
{
	be_config_.dpc = dpc;
	be_config_.dpc.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DPC;
}

void BackEnd::SetGeq(pisp_be_geq_config const &geq)
{
	be_config_.geq = geq;
	be_config_.geq.slope_sharper &= (PISP_BE_GEQ_SLOPE | PISP_BE_GEQ_SHARPER);
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_GEQ;
}

void BackEnd::SetTdnInputFormat(pisp_image_format_config const &tdn_input_format)
{
	be_config_.tdn_input_format = tdn_input_format;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN_INPUT; // TDN input address will always be written
	finalise_tiling_ = true;
}

void BackEnd::SetTdnDecompress(pisp_decompress_config const &tdn_decompress)
{
	be_config_.tdn_decompress = tdn_decompress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS;
}

void BackEnd::SetTdn(pisp_be_tdn_config const &tdn)
{
	be_config_.tdn = tdn;
	be_config_.tdn.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN;
}

void BackEnd::GetTdn(pisp_be_tdn_config &tdn) const
{
	tdn = be_config_.tdn;
}

void BackEnd::SetTdnCompress(pisp_compress_config const &tdn_compress)
{
	be_config_.tdn_compress = tdn_compress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN_COMPRESS;
}

void BackEnd::SetTdnOutputFormat(pisp_image_format_config const &tdn_output_format)
{
	be_config_.tdn_output_format = tdn_output_format;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN_OUTPUT; // TDN output address will always be written
	finalise_tiling_ = true;
}

void BackEnd::GetTdnOutputFormat(pisp_image_format_config &tdn_output_format) const
{
	tdn_output_format = be_config_.tdn_output_format;
}

void BackEnd::SetSdn(pisp_be_sdn_config const &sdn)
{
	be_config_.sdn = sdn;
	be_config_.sdn.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_SDN;
}

void BackEnd::SetBlc(pisp_bla_config const &blc)
{
	be_config_.blc = blc;
	be_config_.blc.pad[0] = be_config_.blc.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_BLC;
}

void BackEnd::SetStitchInputFormat(pisp_image_format_config const &stitch_input_format)
{
	be_config_.stitch_input_format = stitch_input_format;
	be_config_.stitch.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH_INPUT;
	finalise_tiling_ = true;
}

void BackEnd::GetStitchInputFormat(pisp_image_format_config &stitch_input_format) const
{
	stitch_input_format = be_config_.stitch_input_format;
}

void BackEnd::SetStitchDecompress(pisp_decompress_config const &stitch_decompress)
{
	be_config_.stitch_decompress = stitch_decompress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS;
}

void BackEnd::SetStitch(pisp_be_stitch_config const &stitch)
{
	be_config_.stitch = stitch;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH;
}

void BackEnd::SetStitchCompress(pisp_compress_config const &stitch_compress)
{
	be_config_.stitch_compress = stitch_compress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH_COMPRESS;
}

void BackEnd::SetStitchOutputFormat(pisp_image_format_config const &stitch_output_format)
{
	be_config_.stitch_output_format = stitch_output_format;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH_OUTPUT;
	finalise_tiling_ = true;
}

void BackEnd::GetStitchOutputFormat(pisp_image_format_config &stitch_output_format) const
{
	stitch_output_format = be_config_.stitch_output_format;
}

void BackEnd::SetCdn(pisp_be_cdn_config const &cdn)
{
	be_config_.cdn = cdn;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_CDN;
}

void BackEnd::SetWbg(pisp_wbg_config const &wbg)
{
	be_config_.wbg = wbg;
	be_config_.wbg.pad[0] = be_config_.wbg.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_WBG;
}

void BackEnd::SetLsc(pisp_be_lsc_config const &lsc, pisp_be_lsc_extra lsc_extra)
{
	// Should not need a finalise_tile if only the cell coefficients have changed.
	finalise_tiling_ |= be_config_.lsc.grid_step_x != lsc.grid_step_x || be_config_.lsc.grid_step_y != lsc.grid_step_y;
	be_config_.lsc = lsc;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_LSC;
	be_config_.lsc_extra = lsc_extra;
}

void BackEnd::SetCac(pisp_be_cac_config const &cac, pisp_be_cac_extra cac_extra)
{
	finalise_tiling_ |= be_config_.cac.grid_step_x != cac.grid_step_x || be_config_.cac.grid_step_y != cac.grid_step_y;
	be_config_.cac = cac;
	be_config_.cac_extra = cac_extra;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_CAC;
}

void BackEnd::SetDebin(pisp_be_debin_config const &debin)
{
	be_config_.debin = debin;
	be_config_.debin.pad[0] = be_config_.debin.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DEBIN;
}

void BackEnd::SetTonemap(pisp_be_tonemap_config const &tonemap)
{
	be_config_.tonemap = tonemap;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TONEMAP;
}

void BackEnd::SetDemosaic(pisp_be_demosaic_config const &demosaic)
{
	be_config_.demosaic = demosaic;
	be_config_.demosaic.pad[0] = be_config_.demosaic.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DEMOSAIC;
}

void BackEnd::GetDemosaic(pisp_be_demosaic_config &demosaic) const
{
	demosaic = be_config_.demosaic;
}

void BackEnd::SetCcm(pisp_be_ccm_config const &ccm)
{
	be_config_.ccm = ccm;
	be_config_.ccm.pad[0] = be_config_.ccm.pad[1] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_CCM;
}

void BackEnd::SetSatControl(pisp_be_sat_control_config const &sat_control)
{
	be_config_.sat_control = sat_control;
	be_config_.sat_control.pad = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_SAT_CONTROL;
}

void BackEnd::SetYcbcr(pisp_be_ccm_config const &ycbcr)
{
	be_config_.ycbcr = ycbcr;
	be_config_.ycbcr.pad[0] = be_config_.ycbcr.pad[1] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_YCBCR;
}

void BackEnd::SetFalseColour(pisp_be_false_colour_config const &false_colour)
{
	be_config_.false_colour = false_colour;
	be_config_.false_colour.pad[0] = be_config_.false_colour.pad[1] = be_config_.false_colour.pad[2] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_FALSE_COLOUR;
}

void BackEnd::SetSharpen(pisp_be_sharpen_config const &sharpen)
{
	be_config_.sharpen = sharpen;
	be_config_.sharpen.pad0[0] = be_config_.sharpen.pad0[1] = be_config_.sharpen.pad0[2] = 0;
	be_config_.sharpen.pad1[0] = be_config_.sharpen.pad1[1] = be_config_.sharpen.pad1[2] = 0;
	be_config_.sharpen.pad2[0] = be_config_.sharpen.pad2[1] = be_config_.sharpen.pad2[2] = 0;
	be_config_.sharpen.pad3[0] = be_config_.sharpen.pad3[1] = be_config_.sharpen.pad3[2] = 0;
	be_config_.sharpen.pad4[0] = be_config_.sharpen.pad4[1] = be_config_.sharpen.pad4[2] = 0;
	be_config_.sharpen.pad5 = be_config_.sharpen.pad6 = be_config_.sharpen.pad7 = be_config_.sharpen.pad8 = be_config_.sharpen.pad9 = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_SHARPEN;
}

void BackEnd::SetShFcCombine(pisp_be_sh_fc_combine_config const &sh_fc_combine)
{
	be_config_.sh_fc_combine = sh_fc_combine;
	be_config_.sh_fc_combine.pad = 0;
	be_config_.dirty_flags_extra |= PISP_BE_DIRTY_SH_FC_COMBINE;
}

void BackEnd::SetYcbcrInverse(pisp_be_ccm_config const &ycbcr_inverse)
{
	be_config_.ycbcr_inverse = ycbcr_inverse;
	be_config_.ycbcr_inverse.pad[0] = be_config_.ycbcr_inverse.pad[1] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_YCBCR_INVERSE;
}

void BackEnd::SetGamma(pisp_be_gamma_config const &gamma)
{
	be_config_.gamma = gamma;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_GAMMA;
}

void BackEnd::GetGamma(pisp_be_gamma_config &gamma)
{
	gamma = be_config_.gamma;
}

void BackEnd::SetCrop(pisp_be_crop_config const &crop)
{
	be_config_.crop = crop;
	be_config_.dirty_flags_extra |= PISP_BE_DIRTY_CROP;
	retile_ = true;
}

void BackEnd::SetCsc(unsigned int i, pisp_be_ccm_config const &csc)
{
	be_config_.csc[i] = csc;
	be_config_.dirty_flags_bayer |= PISP_BE_RGB_ENABLE_CSC(i);
}

void BackEnd::SetDownscale(unsigned int i, pisp_be_downscale_config const &downscale, pisp_be_downscale_extra const &downscale_extra)
{
	be_config_.downscale[i] = downscale;
	be_config_.downscale_extra[i] = downscale_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_DOWNSCALE(i);
	retile_ = true;
}

void BackEnd::SetDownscale(unsigned int i, pisp_be_downscale_extra const &downscale_extra)
{
	be_config_.downscale_extra[i] = downscale_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_DOWNSCALE(i);
	retile_ = true;
}

void BackEnd::SetResample(unsigned int i, pisp_be_resample_config const &resample, pisp_be_resample_extra const &resample_extra)
{
	be_config_.resample[i] = resample;
	be_config_.resample_extra[i] = resample_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_RESAMPLE(i);
	retile_ = true;
}

void BackEnd::SetResample(unsigned int i, pisp_be_resample_extra const &resample_extra)
{
	be_config_.resample_extra[i] = resample_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_RESAMPLE(i);
	retile_ = true;
}

void BackEnd::SetOutputFormat(unsigned int i, pisp_be_output_format_config const &output_format)
{
	ASSERT(i < variant_.backEndNumBranches(0));
	be_config_.output_format[i] = output_format;

	if (output_format.image.format & PISP_IMAGE_FORMAT_INTEGRAL_IMAGE) {
		// If this is an integral image request, we must constrain the format parameters!
		be_config_.output_format[i].image.format = PISP_IMAGE_FORMAT_INTEGRAL_IMAGE +
							   PISP_IMAGE_FORMAT_PLANARITY_PLANAR +
							   PISP_IMAGE_FORMAT_SAMPLING_444 +
							   (output_format.image.format & PISP_IMAGE_FORMAT_SHIFT_MASK) +
							   (output_format.image.format & PISP_IMAGE_FORMAT_THREE_CHANNEL);
	}
	be_config_.output_format[i].pad[0] = be_config_.output_format[i].pad[1] = be_config_.output_format[i].pad[2] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_OUTPUT(i);
	// Should only need a retile if the transform has changed, othwise a finalise_tile will do.
	retile_ = true;
}

void BackEnd::SetHog(pisp_be_hog_config const &hog)
{
	be_config_.hog = hog;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_HOG;
	finalise_tiling_ = true;
}

void BackEnd::GetOutputFormat(unsigned int i, pisp_be_output_format_config &output_format) const
{
	ASSERT(i < variant_.backEndNumBranches(0));
	output_format = be_config_.output_format[i];
}
