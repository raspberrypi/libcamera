// Implementation of the PiSP Back End driver.
#include "backend.h"

#include <libcamera/base/log.h>

using namespace libcamera;
using namespace PiSP;

LOG_DEFINE_CATEGORY(PISPBE)

// HoG feature constants
constexpr unsigned int HogCellSize = 8;

// Gives software a chance to initialise certain parameters to sensible non-zero values.
static void initialise_config(pisp_be_config &config);

BackEnd::BackEnd(Config const &config, PiSPVariant &variant)
	: config_(config), variant_(variant), retile_(true), finalise_tiling_(true)
{
	int max_tile_width = variant_.backEndMaxTileWidth(0);

	// Just complain if the max tile width is too large but don't change it - this allows software simulations still
	// to run in full image (or "single tile") mode.
	if (config_.max_tile_width > max_tile_width)
		LOG(PISPBE, Warning) << "Configured max tile width " << config_.max_tile_width << " exceeds " << max_tile_width;

	initialise_config(be_config_);
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

void BackEnd::SetDecompress(PISP_BE_DECOMPRESS_CONFIG_T const &decompress)
{
	be_config_.decompress = decompress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DECOMPRESS;
}

void BackEnd::SetDpc(PISP_BE_DPC_CONFIG_T const &dpc)
{
	be_config_.dpc = dpc;
	be_config_.dpc.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DPC;
}

void BackEnd::SetGeq(PISP_BE_GEQ_CONFIG_T const &geq)
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

void BackEnd::SetTdnDecompress(PISP_BE_TDN_DECOMPRESS_CONFIG_T const &tdn_decompress)
{
	be_config_.tdn_decompress = tdn_decompress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN_DECOMPRESS;
}

void BackEnd::SetTdn(PISP_BE_TDN_CONFIG_T const &tdn)
{
	be_config_.tdn = tdn;
	be_config_.tdn.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TDN;
}

void BackEnd::GetTdn(PISP_BE_TDN_CONFIG_T &tdn) const
{
	tdn = be_config_.tdn;
}

void BackEnd::SetTdnCompress(PISP_BE_TDN_COMPRESS_CONFIG_T const &tdn_compress)
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

void BackEnd::SetSdn(PISP_BE_SDN_CONFIG_T const &sdn)
{
	be_config_.sdn = sdn;
	be_config_.sdn.pad = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_SDN;
}

void BackEnd::SetBlc(PISP_BE_BLC_CONFIG_T const &blc)
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

void BackEnd::SetStitchDecompress(PISP_BE_STITCH_DECOMPRESS_CONFIG_T const &stitch_decompress)
{
	be_config_.stitch_decompress = stitch_decompress;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH_DECOMPRESS;
}

void BackEnd::SetStitch(PISP_BE_STITCH_CONFIG_T const &stitch)
{
	be_config_.stitch = stitch;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_STITCH;
}

void BackEnd::SetStitchCompress(PISP_BE_STITCH_COMPRESS_CONFIG_T const &stitch_compress)
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

void BackEnd::SetCdn(PISP_BE_CDN_CONFIG_T const &cdn)
{
	be_config_.cdn = cdn;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_CDN;
}

void BackEnd::SetWbg(PISP_BE_WBG_CONFIG_T const &wbg)
{
	be_config_.wbg = wbg;
	be_config_.wbg.pad[0] = be_config_.wbg.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_WBG;
}

void BackEnd::SetLsc(PISP_BE_LSC_CONFIG_T const &lsc, PISP_BE_LSC_EXTRA_T lsc_extra)
{
	// Should not need a finalise_tile if only the cell coefficients have changed.
	finalise_tiling_ |= be_config_.lsc.grid_step_x != lsc.grid_step_x || be_config_.lsc.grid_step_y != lsc.grid_step_y;
	be_config_.lsc = lsc;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_LSC;
	be_config_.lsc_extra = lsc_extra;
}

void BackEnd::SetCac(PISP_BE_CAC_CONFIG_T const &cac, PISP_BE_CAC_EXTRA_T cac_extra)
{
	finalise_tiling_ |= be_config_.cac.grid_step_x != cac.grid_step_x || be_config_.cac.grid_step_y != cac.grid_step_y;
	be_config_.cac = cac;
	be_config_.cac_extra = cac_extra;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_CAC;
}

void BackEnd::SetDebin(PISP_BE_DEBIN_CONFIG_T const &debin)
{
	be_config_.debin = debin;
	be_config_.debin.pad[0] = be_config_.debin.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DEBIN;
}

void BackEnd::SetTonemap(PISP_BE_TONEMAP_CONFIG_T const &tonemap)
{
	be_config_.tonemap = tonemap;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_TONEMAP;
}

void BackEnd::SetDemosaic(PISP_BE_DEMOSAIC_CONFIG_T const &demosaic)
{
	be_config_.demosaic = demosaic;
	be_config_.demosaic.pad[0] = be_config_.demosaic.pad[1] = 0;
	be_config_.dirty_flags_bayer |= PISP_BE_BAYER_ENABLE_DEMOSAIC;
}

void BackEnd::GetDemosaic(PISP_BE_DEMOSAIC_CONFIG_T &demosaic) const
{
	demosaic = be_config_.demosaic;
}

void BackEnd::SetCcm(PISP_BE_CCM_CONFIG_T const &ccm)
{
	be_config_.ccm = ccm;
	be_config_.ccm.pad[0] = be_config_.ccm.pad[1] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_CCM;
}

void BackEnd::SetSatControl(PISP_BE_SAT_CONTROL_CONFIG_T const &sat_control)
{
	be_config_.sat_control = sat_control;
	be_config_.sat_control.pad = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_SAT_CONTROL;
}

void BackEnd::SetYcbcr(PISP_BE_YCBCR_CONFIG_T const &ycbcr)
{
	be_config_.ycbcr = ycbcr;
	be_config_.ycbcr.pad[0] = be_config_.ycbcr.pad[1] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_YCBCR;
}

void BackEnd::SetFalseColour(PISP_BE_FALSE_COLOUR_CONFIG_T const &false_colour)
{
	be_config_.false_colour = false_colour;
	be_config_.false_colour.pad[0] = be_config_.false_colour.pad[1] = be_config_.false_colour.pad[2] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_FALSE_COLOUR;
}

void BackEnd::SetSharpen(PISP_BE_SHARPEN_CONFIG_T const &sharpen)
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

void BackEnd::SetShFcCombine(PISP_BE_SH_FC_COMBINE_CONFIG_T const &sh_fc_combine)
{
	be_config_.sh_fc_combine = sh_fc_combine;
	be_config_.sh_fc_combine.pad = 0;
	be_config_.dirty_flags_extra |= PISP_BE_DIRTY_SH_FC_COMBINE;
}

void BackEnd::SetYcbcrInverse(PISP_BE_YCBCR_INVERSE_CONFIG_T const &ycbcr_inverse)
{
	be_config_.ycbcr_inverse = ycbcr_inverse;
	be_config_.ycbcr_inverse.pad[0] = be_config_.ycbcr_inverse.pad[1] = 0;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_YCBCR_INVERSE;
}

void BackEnd::SetGamma(PISP_BE_GAMMA_CONFIG_T const &gamma)
{
	be_config_.gamma = gamma;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_GAMMA;
}

void BackEnd::GetGamma(PISP_BE_GAMMA_CONFIG_T &gamma)
{
	gamma = be_config_.gamma;
}

void BackEnd::SetCrop(PISP_BE_CROP_CONFIG_T const &crop)
{
	be_config_.crop = crop;
	be_config_.dirty_flags_extra |= PISP_BE_DIRTY_CROP;
	retile_ = true;
}

void BackEnd::SetCsc(int i, PISP_BE_CSC_CONFIG_T const &csc)
{
	be_config_.csc[i] = csc;
	be_config_.dirty_flags_bayer |= PISP_BE_RGB_ENABLE_CSC(i);
}

void BackEnd::SetDownscale(int i, PISP_BE_DOWNSCALE_CONFIG_T const &downscale, PISP_BE_DOWNSCALE_EXTRA_T const &downscale_extra)
{
	be_config_.downscale[i] = downscale;
	be_config_.downscale_extra[i] = downscale_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_DOWNSCALE(i);
	retile_ = true;
}

void BackEnd::SetDownscale(int i, PISP_BE_DOWNSCALE_EXTRA_T const &downscale_extra)
{
	be_config_.downscale_extra[i] = downscale_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_DOWNSCALE(i);
	retile_ = true;
}

void BackEnd::SetResample(int i, PISP_BE_RESAMPLE_CONFIG_T const &resample, PISP_BE_RESAMPLE_EXTRA_T const &resample_extra)
{
	be_config_.resample[i] = resample;
	be_config_.resample_extra[i] = resample_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_RESAMPLE(i);
	retile_ = true;
}

void BackEnd::SetResample(int i, PISP_BE_RESAMPLE_EXTRA_T const &resample_extra)
{
	be_config_.resample_extra[i] = resample_extra;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_RESAMPLE(i);
	retile_ = true;
}

void BackEnd::SetOutputFormat(int i, PISP_BE_OUTPUT_FORMAT_CONFIG_T const &output_format)
{
	assert(i < PISP_BACK_END_NUM_OUTPUTS);
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

void BackEnd::SetHog(PISP_BE_HOG_CONFIG_T const &hog)
{
	be_config_.hog = hog;
	be_config_.dirty_flags_rgb |= PISP_BE_RGB_ENABLE_HOG;
	finalise_tiling_ = true;
}

void BackEnd::GetOutputFormat(int i, PISP_BE_OUTPUT_FORMAT_CONFIG_T &output_format) const
{
	assert(i < PISP_BACK_END_NUM_OUTPUTS);
	output_format = be_config_.output_format[i];
}

{
	static const std::thread::id noid;
	if (thread_id_ != noid) {
		bool is_same = (thread_id_ == std::this_thread::get_id());
		if (from_backend_thread ^ is_same)
			throw std::logic_error("BackEnd API called from an invalid thread context.");
	}
}
