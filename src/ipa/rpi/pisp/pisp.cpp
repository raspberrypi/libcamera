/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * pisp.cpp - Raspberry Pi PiSP IPA
 */
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <sys/mman.h>
#include <utility>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libipa/pwl.h>

#include "libpisp/backend/backend.hpp"
#include "libpisp/frontend/frontend.hpp"

#include "common/ipa_base.h"
#include "controller/af_status.h"
#include "controller/agc_algorithm.h"
#include "controller/alsc_status.h"
#include "controller/awb_algorithm.h"
#include "controller/awb_status.h"
#include "controller/black_level_algorithm.h"
#include "controller/black_level_status.h"
#include "controller/cac_status.h"
#include "controller/ccm_status.h"
#include "controller/contrast_status.h"
#include "controller/denoise_algorithm.h"
#include "controller/denoise_status.h"
#include "controller/dpc_status.h"
#include "controller/geq_status.h"
#include "controller/hdr_status.h"
#include "controller/lux_status.h"
#include "controller/noise_status.h"
#include "controller/saturation_status.h"
#include "controller/sharpen_status.h"
#include "controller/stitch_status.h"
#include "controller/tonemap_status.h"

using namespace std::literals::chrono_literals;

namespace libcamera {

LOG_DECLARE_CATEGORY(IPARPI)

namespace {

constexpr unsigned int NumLscCells = PISP_BE_LSC_GRID_SIZE;
constexpr unsigned int NumLscVertexes = NumLscCells + 1;

inline int32_t clampField(double value, std::size_t fieldBits, std::size_t fracBits = 0,
			  bool isSigned = false, const char *desc = nullptr)
{
	ASSERT(fracBits <= fieldBits && fieldBits <= 32);

	int min = -(isSigned << (fieldBits - 1));
	int max = (1 << (fieldBits - isSigned)) - 1;
	int32_t val =
		std::clamp<int32_t>(std::round(value * (1 << fracBits)), min, max);

	if (desc && val / (1 << fracBits) != value)
		LOG(IPARPI, Warning)
			<< desc << " rounded/clamped to " << val / (1 << fracBits);

	return val;
}

int generateLut(const ipa::Pwl &pwl, uint32_t *lut, std::size_t lutSize,
		unsigned int SlopeBits = 14, unsigned int PosBits = 16)
{
	if (pwl.empty())
		return -EINVAL;

	int lastY = 0;
	for (unsigned int i = 0; i < lutSize; i++) {
		int x, y;
		if (i < 32)
			x = i * 512;
		else if (i < 48)
			x = (i - 32) * 1024 + 16384;
		else
			x = std::min(65535u, (i - 48) * 2048 + 32768);

		y = pwl.eval(x);
		if (y < 0 || (i && y < lastY)) {
			LOG(IPARPI, Error)
				<< "Malformed PWL for Gamma, disabling!";
			return -1;
		}

		if (i) {
			unsigned int slope = y - lastY;
			if (slope >= (1u << SlopeBits)) {
				slope = (1u << SlopeBits) - 1;
				LOG(IPARPI, Info)
					<< ("Maximum Gamma slope exceeded, adjusting!");
				y = lastY + slope;
			}
			lut[i - 1] |= slope << PosBits;
		}

		lut[i] = y;
		lastY = y;
	}

	return 0;
}

void packLscLut(uint32_t packed[NumLscVertexes][NumLscVertexes],
		double const rgb[3][NumLscVertexes][NumLscVertexes])
{
	for (unsigned int y = 0; y < NumLscVertexes; ++y) {
		for (unsigned int x = 0; x < NumLscVertexes; ++x) {
			/* Jointly encode RGB gains in one of 4 ranges: [0.5:1.5), [0:2), [0:4), [0:8) */
			double lo = std::min({ rgb[0][y][x], rgb[1][y][x], rgb[2][y][x] });
			double hi = std::max({ rgb[0][y][x], rgb[1][y][x], rgb[2][y][x] });
			uint32_t range;
			double scale, offset;
			if (lo >= 0.5 && hi < 1.5) {
				range = 0;
				scale = 1024.0;
				offset = -511.5;
			} else if (hi < 2.0) {
				range = 1;
				scale = 512.0;
				offset = 0.5;
			} else if (hi < 4.0) {
				range = 2;
				scale = 256.0;
				offset = 0.5;
			} else {
				range = 3;
				scale = 128.0;
				offset = 0.5;
			}
			int r = clampField(offset + scale * rgb[0][y][x], 10);
			int g = clampField(offset + scale * rgb[1][y][x], 10);
			int b = clampField(offset + scale * rgb[2][y][x], 10);
			packed[y][x] = (range << 30) | (b << 20) | (g << 10) | r;
		}
	}
}

/*
 * Resamples a srcW x srcH table with central sampling to destW x destH with
 * corner sampling.
 */
void resampleTable(double *dest, int destW, int destH, double const *src,
		   int srcW, int srcH)
{
	/*
	 * Precalculate and cache the x sampling locations and phases to
	 * save recomputing them on every row.
	 */
	ASSERT(destW > 1 && destH > 1 && destW <= 64);
	int xLo[64], xHi[64];
	double xf[64];
	double x = -0.5, xInc = srcW / (destW - 1);
	for (int i = 0; i < destW; i++, x += xInc) {
		xLo[i] = floor(x);
		xf[i] = x - xLo[i];
		xHi[i] = xLo[i] < (srcW - 1) ? (xLo[i] + 1) : (srcW - 1);
		xLo[i] = xLo[i] > 0 ? xLo[i] : 0;
	}

	/* Now march over the output table generating the new values. */
	double y = -0.5, yInc = srcH / (destH - 1);
	for (int j = 0; j < destH; j++, y += yInc) {
		int yLo = floor(y);
		double yf = y - yLo;
		int yHi = yLo < (srcH - 1) ? (yLo + 1) : (srcH - 1);
		yLo = yLo > 0 ? yLo : 0;
		double const *rowAbove = src + yLo * srcW;
		double const *rowBelow = src + yHi * srcW;
		for (int i = 0; i < destW; i++) {
			double above = rowAbove[xLo[i]] * (1 - xf[i]) +
				       rowAbove[xHi[i]] * xf[i];
			double below = rowBelow[xLo[i]] * (1 - xf[i]) +
				       rowBelow[xHi[i]] * xf[i];
			*(dest++) = above * (1 - yf) + below * yf;
		}
	}
}

} /* namespace */

using ::libpisp::BackEnd;
using ::libpisp::FrontEnd;

namespace ipa::RPi {

class IpaPiSP final : public IpaBase
{
public:
	IpaPiSP()
		: IpaBase(), fe_(nullptr), be_(nullptr)
	{
	}

	~IpaPiSP()
	{
		if (fe_)
			munmap(fe_, sizeof(FrontEnd));
		if (be_)
			munmap(be_, sizeof(BackEnd));
	}

private:
	int32_t platformInit(const InitParams &params, InitResult *result) override;
	int32_t platformStart(const ControlList &controls, StartResult *result) override;
	int32_t platformConfigure(const ConfigParams &params, ConfigResult *result) override;

	void platformPrepareIsp(const PrepareParams &params,
				RPiController::Metadata &rpiMetadata) override;
	RPiController::StatisticsPtr platformProcessStats(Span<uint8_t> mem) override;

	void handleControls(const ControlList &controls) override;

	void applyWBG(const AwbStatus *awbStatus, const AgcPrepareStatus *agcStatus,
		      pisp_be_global_config &global);
	void applyDgOnly(const AgcPrepareStatus *agcPrepareStatus, pisp_be_global_config &global);
	void applyCAC(const CacStatus *cacStatus, pisp_be_global_config &global);
	void applyContrast(const ContrastStatus *contrastStatus,
			   pisp_be_global_config &global);
	void applyCCM(const CcmStatus *ccmStatus, pisp_be_global_config &global);
	void applyBlackLevel(const BlackLevelStatus *blackLevelStatus,
			     pisp_be_global_config &global);
	void applyLensShading(const AlscStatus *alscStatus,
			      pisp_be_global_config &global);
	void applyDPC(const DpcStatus *dpcStatus, pisp_be_global_config &global);
	void applySdn(const SdnStatus *sdnStatus, pisp_be_global_config &global);
	void applyTdn(const TdnStatus *tdnStatus, const DeviceStatus *deviceStatus,
		      pisp_be_global_config &global);
	void applyCdn(const CdnStatus *cdnStatus, pisp_be_global_config &global);
	void applyGeq(const GeqStatus *geqStatus, pisp_be_global_config &global);
	void applySaturation(const SaturationStatus *geqStatus,
			     pisp_be_global_config &global);
	void applySharpen(const SharpenStatus *sharpenStatus,
			  pisp_be_global_config &global);
	bool applyStitch(const StitchStatus *stitchStatus, const DeviceStatus *deviceStatus,
			 const AgcStatus *agcStatus, pisp_be_global_config &global);
	void applyTonemap(const TonemapStatus *tonemapStatus,
			  pisp_be_global_config &global);
	void applyFocusStats(const NoiseStatus *noiseStatus);
	void applyAF(const struct AfStatus *afStatus, ControlList &lensCtrls);

	void setDefaultConfig();
	void setStatsAndDebin();
	void setHistogramWeights();

	/* Frontend/Backend objects passed in from the pipeline handler. */
	SharedFD feFD_;
	SharedFD beFD_;
	FrontEnd *fe_;
	BackEnd *be_;

	/* TDN/HDR runtime need the following state. */
	bool tdnReset_;
	utils::Duration lastExposure_;
	std::map<std::string, utils::Duration> lastStitchExposures_;
	HdrStatus lastStitchHdrStatus_;
};

int32_t IpaPiSP::platformInit(const InitParams &params,
			      [[maybe_unused]] InitResult *result)
{
	const std::string &target = controller_.getTarget();
	if (target != "pisp") {
		LOG(IPARPI, Error)
			<< "Tuning data file target returned \"" << target << "\""
			<< ", expected \"pisp\"";
		return -EINVAL;
	}

	/* Acquire the Frontend and Backend objects. */
	feFD_ = std::move(params.fe);
	beFD_ = std::move(params.be);

	if (!feFD_.isValid() || !beFD_.isValid()) {
		LOG(IPARPI, Error) << "Invalid FE/BE handles!";
		return -ENODEV;
	}

	fe_ = static_cast<FrontEnd *>(mmap(nullptr, sizeof(FrontEnd),
					   PROT_READ | PROT_WRITE, MAP_SHARED,
					   feFD_.get(), 0));
	be_ = static_cast<BackEnd *>(mmap(nullptr, sizeof(BackEnd),
					  PROT_READ | PROT_WRITE, MAP_SHARED,
					  beFD_.get(), 0));

	if (!fe_ || !be_) {
		LOG(IPARPI, Error) << "Unable to map FE/BE handles!";
		return -ENODEV;
	}

	setDefaultConfig();

	return 0;
}

int32_t IpaPiSP::platformStart([[maybe_unused]] const ControlList &controls,
			       [[maybe_unused]] StartResult *result)
{
	tdnReset_ = true;

	/* Cause the stitch block to be reset correctly. */
	lastStitchHdrStatus_ = HdrStatus();

	return 0;
}

int32_t IpaPiSP::platformConfigure([[maybe_unused]] const ConfigParams &params,
				   [[maybe_unused]] ConfigResult *result)
{
	setStatsAndDebin();
	return 0;
}

void IpaPiSP::platformPrepareIsp([[maybe_unused]] const PrepareParams &params,
				 RPiController::Metadata &rpiMetadata)
{
	std::scoped_lock<RPiController::Metadata> l(rpiMetadata);

	pisp_be_global_config global;
	be_->GetGlobal(global);

	global.bayer_enables &= ~(PISP_BE_BAYER_ENABLE_BLC + PISP_BE_BAYER_ENABLE_WBG +
				  PISP_BE_BAYER_ENABLE_GEQ + PISP_BE_BAYER_ENABLE_LSC +
				  PISP_BE_BAYER_ENABLE_SDN + PISP_BE_BAYER_ENABLE_CDN +
				  PISP_BE_BAYER_ENABLE_TDN_OUTPUT + PISP_BE_BAYER_ENABLE_TDN_INPUT +
				  PISP_BE_BAYER_ENABLE_STITCH_INPUT + PISP_BE_BAYER_ENABLE_STITCH_OUTPUT +
				  PISP_BE_BAYER_ENABLE_STITCH + PISP_BE_BAYER_ENABLE_TONEMAP);
	/* We leave the YCbCr and inverse conversion enabled in case of false colour or sharpening. */
	global.rgb_enables &= ~(PISP_BE_RGB_ENABLE_GAMMA + PISP_BE_RGB_ENABLE_CCM +
				PISP_BE_RGB_ENABLE_SHARPEN + PISP_BE_RGB_ENABLE_SAT_CONTROL);

	NoiseStatus *noiseStatus = rpiMetadata.getLocked<NoiseStatus>("noise.status");
	AgcPrepareStatus *agcPrepareStatus = rpiMetadata.getLocked<AgcPrepareStatus>("agc.prepare_status");

	{
		/* All Frontend config goes first, we do not want to hold the FE lock for long! */
		std::scoped_lock<FrontEnd> lf(*fe_);

		if (noiseStatus)
			applyFocusStats(noiseStatus);

		BlackLevelStatus *blackLevelStatus =
			rpiMetadata.getLocked<BlackLevelStatus>("black_level.status");
		if (blackLevelStatus)
			applyBlackLevel(blackLevelStatus, global);

		AwbStatus *awbStatus = rpiMetadata.getLocked<AwbStatus>("awb.status");
		if (awbStatus && agcPrepareStatus) {
			/* Applies digital gain as well. */
			applyWBG(awbStatus, agcPrepareStatus, global);
		} else if (agcPrepareStatus) {
			/* Mono sensor fallback for digital gain. */
			applyDgOnly(agcPrepareStatus, global);
		}
	}

	CacStatus *cacStatus = rpiMetadata.getLocked<CacStatus>("cac.status");
	if (cacStatus)
		applyCAC(cacStatus, global);

	ContrastStatus *contrastStatus =
		rpiMetadata.getLocked<ContrastStatus>("contrast.status");
	if (contrastStatus)
		applyContrast(contrastStatus, global);

	CcmStatus *ccmStatus = rpiMetadata.getLocked<CcmStatus>("ccm.status");
	if (ccmStatus)
		applyCCM(ccmStatus, global);

	AlscStatus *alscStatus = rpiMetadata.getLocked<AlscStatus>("alsc.status");
	if (alscStatus)
		applyLensShading(alscStatus, global);

	DpcStatus *dpcStatus = rpiMetadata.getLocked<DpcStatus>("dpc.status");
	if (dpcStatus)
		applyDPC(dpcStatus, global);

	SdnStatus *sdnStatus = rpiMetadata.getLocked<SdnStatus>("sdn.status");
	if (sdnStatus)
		applySdn(sdnStatus, global);

	DeviceStatus *deviceStatus = rpiMetadata.getLocked<DeviceStatus>("device.status");
	TdnStatus *tdnStatus = rpiMetadata.getLocked<TdnStatus>("tdn.status");
	if (tdnStatus && deviceStatus)
		applyTdn(tdnStatus, deviceStatus, global);

	CdnStatus *cdnStatus = rpiMetadata.getLocked<CdnStatus>("cdn.status");
	if (cdnStatus)
		applyCdn(cdnStatus, global);

	GeqStatus *geqStatus = rpiMetadata.getLocked<GeqStatus>("geq.status");
	if (geqStatus)
		applyGeq(geqStatus, global);

	SaturationStatus *saturationStatus =
		rpiMetadata.getLocked<SaturationStatus>("saturation.status");
	if (saturationStatus)
		applySaturation(saturationStatus, global);

	SharpenStatus *sharpenStatus = rpiMetadata.getLocked<SharpenStatus>("sharpen.status");
	if (sharpenStatus)
		applySharpen(sharpenStatus, global);

	StitchStatus *stitchStatus = rpiMetadata.getLocked<StitchStatus>("stitch.status");
	if (stitchStatus) {
		/*
		 * Note that it's the *delayed* AGC status that contains the HDR mode/channel
		 * info that pertains to this frame!
		 */
		AgcStatus *agcStatus = rpiMetadata.getLocked<AgcStatus>("agc.delayed_status");
		/* prepareIsp() will fetch this value. Maybe pass it back differently? */
		stitchSwapBuffers_ = applyStitch(stitchStatus, deviceStatus, agcStatus, global);
	} else
		lastStitchHdrStatus_ = HdrStatus();

	TonemapStatus *tonemapStatus = rpiMetadata.getLocked<TonemapStatus>("tonemap.status");
	if (tonemapStatus)
		applyTonemap(tonemapStatus, global);

	be_->SetGlobal(global);

	/* Save this for TDN and HDR on the next frame. */
	lastExposure_ = deviceStatus->exposureTime * deviceStatus->analogueGain;

	/* Lens control */
	const AfStatus *afStatus = rpiMetadata.getLocked<AfStatus>("af.status");
	if (afStatus) {
		ControlList lensctrls(lensCtrls_);
		applyAF(afStatus, lensctrls);
		if (!lensctrls.empty())
			setLensControls.emit(lensctrls);
	}
}

RPiController::StatisticsPtr IpaPiSP::platformProcessStats(Span<uint8_t> mem)
{
	using namespace RPiController;

	const pisp_statistics *stats = reinterpret_cast<pisp_statistics *>(mem.data());

	unsigned int i;
	StatisticsPtr statistics =
		std::make_unique<Statistics>(Statistics::AgcStatsPos::PostWb,
					     Statistics::ColourStatsPos::PreLsc);

	/* RGB histograms are not used, so do not populate them. */
	statistics->yHist = RPiController::Histogram(stats->agc.histogram,
						     PISP_AGC_STATS_NUM_BINS);

	statistics->awbRegions.init({ PISP_AWB_STATS_SIZE, PISP_AWB_STATS_SIZE });
	for (i = 0; i < statistics->awbRegions.numRegions(); i++)
		statistics->awbRegions.set(i, { { stats->awb.zones[i].R_sum,
						  stats->awb.zones[i].G_sum,
						  stats->awb.zones[i].B_sum },
						stats->awb.zones[i].counted, 0 });

	/* AGC region sums only get collected on floating zones. */
	statistics->agcRegions.init({ 0, 0 }, PISP_FLOATING_STATS_NUM_ZONES);
	for (i = 0; i < statistics->agcRegions.numRegions(); i++)
		statistics->agcRegions.setFloating(i,
						   { { 0, 0, 0, stats->agc.floating[i].Y_sum },
						     stats->agc.floating[i].counted, 0 });

	statistics->focusRegions.init({ PISP_CDAF_STATS_SIZE, PISP_CDAF_STATS_SIZE });
	for (i = 0; i < statistics->focusRegions.numRegions(); i++)
		statistics->focusRegions.set(i, { stats->cdaf.foms[i] >> 20, 0, 0 });

	if (statsMetadataOutput_) {
		Span<const uint8_t> statsSpan(reinterpret_cast<const uint8_t *>(stats),
					      sizeof(pisp_statistics));
		libcameraMetadata_.set(controls::rpi::PispStatsOutput, statsSpan);
	}

	return statistics;
}

void IpaPiSP::handleControls(const ControlList &controls)
{
	for (auto const &ctrl : controls) {
		switch (ctrl.first) {
		case controls::HDR_MODE:
		case controls::AE_METERING_MODE:
			setHistogramWeights();
			break;

		case controls::draft::NOISE_REDUCTION_MODE: {
			RPiController::DenoiseAlgorithm *denoise = dynamic_cast<RPiController::DenoiseAlgorithm *>(
				controller_.getAlgorithm("denoise"));

			if (!denoise) {
				LOG(IPARPI, Warning)
					<< "Could not set NOISE_REDUCTION_MODE - no Denoise algorithm";
				return;
			}

			if (ctrl.second.get<int32_t>() == controls::draft::NoiseReductionModeOff)
				denoise->setMode(RPiController::DenoiseMode::Off);
			else
				denoise->setMode(RPiController::DenoiseMode::ColourHighQuality);

			break;
		}
		}
	}
}

void IpaPiSP::applyWBG(const AwbStatus *awbStatus, const AgcPrepareStatus *agcPrepareStatus,
		       pisp_be_global_config &global)
{
	pisp_wbg_config wbg;
	pisp_fe_rgby_config rgby = {};
	double dg = agcPrepareStatus ? agcPrepareStatus->digitalGain : 1.0;

	wbg.gain_r = clampField(dg * awbStatus->gainR, 14, 10);
	wbg.gain_g = clampField(dg * awbStatus->gainG, 14, 10);
	wbg.gain_b = clampField(dg * awbStatus->gainB, 14, 10);

	/*
	 * The YCbCr conversion block should contain the appropriate YCbCr
	 * matrix. We should not rely on the CSC0 block as that might be
	 * programmed for RGB outputs.
	 */
	pisp_be_ccm_config csc;
	be_->GetYcbcr(csc);

	/* The CSC coefficients already have the << 10 scaling applied. */
	rgby.gain_r = clampField(csc.coeffs[0] * awbStatus->gainR, 14);
	rgby.gain_g = clampField(csc.coeffs[1] * awbStatus->gainG, 14);
	rgby.gain_b = clampField(csc.coeffs[2] * awbStatus->gainB, 14);

	LOG(IPARPI, Debug) << "Applying WB R: " << awbStatus->gainR << " B: "
			   << awbStatus->gainB;

	be_->SetWbg(wbg);
	fe_->SetRGBY(rgby);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_WBG;
}

void IpaPiSP::applyDgOnly(const AgcPrepareStatus *agcPrepareStatus, pisp_be_global_config &global)
{
	pisp_wbg_config wbg;

	wbg.gain_r = clampField(agcPrepareStatus->digitalGain, 14, 10);
	wbg.gain_g = clampField(agcPrepareStatus->digitalGain, 14, 10);
	wbg.gain_b = clampField(agcPrepareStatus->digitalGain, 14, 10);

	LOG(IPARPI, Debug) << "Applying DG (only) : " << agcPrepareStatus->digitalGain;

	be_->SetWbg(wbg);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_WBG;
}

void IpaPiSP::applyContrast(const ContrastStatus *contrastStatus,
			    pisp_be_global_config &global)
{
	pisp_be_gamma_config gamma;

	if (!generateLut(contrastStatus->gammaCurve, gamma.lut, PISP_BE_GAMMA_LUT_SIZE)) {
		be_->SetGamma(gamma);
		global.rgb_enables |= PISP_BE_RGB_ENABLE_GAMMA;
	}
}

void IpaPiSP::applyCCM(const CcmStatus *ccmStatus, pisp_be_global_config &global)
{
	pisp_be_ccm_config ccm = {};

	for (unsigned int i = 0; i < 9; i++)
		ccm.coeffs[i] = clampField(ccmStatus->matrix[i], 14, 10, true);

	be_->SetCcm(ccm);
	global.rgb_enables |= PISP_BE_RGB_ENABLE_CCM;
}

void IpaPiSP::applyCAC(const CacStatus *cacStatus, pisp_be_global_config &global)
{
	pisp_be_cac_config cac = {};

	for (int x = 0; x < PISP_BE_CAC_GRID_SIZE + 1; x++) {
		for (int y = 0; y < PISP_BE_CAC_GRID_SIZE + 1; y++) {
			cac.lut[y][x][0][0] = clampField(cacStatus->lutRx[y * (PISP_BE_CAC_GRID_SIZE + 1) + x], 7, 5, true);
			cac.lut[y][x][0][1] = clampField(cacStatus->lutRy[y * (PISP_BE_CAC_GRID_SIZE + 1) + x], 7, 5, true);
			cac.lut[y][x][1][0] = clampField(cacStatus->lutBx[y * (PISP_BE_CAC_GRID_SIZE + 1) + x], 7, 5, true);
			cac.lut[y][x][1][1] = clampField(cacStatus->lutBy[y * (PISP_BE_CAC_GRID_SIZE + 1) + x], 7, 5, true);
		}
	}

	be_->SetCac(cac);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_CAC;
}

void IpaPiSP::applyBlackLevel(const BlackLevelStatus *blackLevelStatus, pisp_be_global_config &global)
{
	uint16_t minBlackLevel = std::min({ blackLevelStatus->blackLevelR, blackLevelStatus->blackLevelG,
					    blackLevelStatus->blackLevelB });
	pisp_bla_config bla;

	/*
	 * Set the Frontend to adjust the black level to the smallest black level
	 * of all channels (in 16-bits).
	 */
	bla.black_level_r = blackLevelStatus->blackLevelR;
	bla.black_level_gr = blackLevelStatus->blackLevelG;
	bla.black_level_gb = blackLevelStatus->blackLevelG;
	bla.black_level_b = blackLevelStatus->blackLevelB;
	bla.output_black_level = minBlackLevel;
	fe_->SetBla(bla);

	/* Frontend Stats and Backend black level correction. */
	bla.black_level_r = bla.black_level_gr =
		bla.black_level_gb = bla.black_level_b = minBlackLevel;
	bla.output_black_level = 0;
	fe_->SetBlc(bla);
	be_->SetBlc(bla);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_BLC;
}

void IpaPiSP::applyLensShading(const AlscStatus *alscStatus,
			       pisp_be_global_config &global)
{
	pisp_be_lsc_extra lscExtra = {};
	pisp_be_lsc_config lsc = {};
	double rgb[3][NumLscVertexes][NumLscVertexes] = {};

	resampleTable(&rgb[0][0][0], NumLscVertexes, NumLscVertexes,
		      alscStatus->r.data(), NumLscCells, NumLscCells);
	resampleTable(&rgb[1][0][0], NumLscVertexes, NumLscVertexes,
		      alscStatus->g.data(), NumLscCells, NumLscCells);
	resampleTable(&rgb[2][0][0], NumLscVertexes, NumLscVertexes,
		      alscStatus->b.data(), NumLscCells, NumLscCells);
	packLscLut(lsc.lut_packed, rgb);
	be_->SetLsc(lsc, lscExtra);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_LSC;
}

void IpaPiSP::applyDPC(const DpcStatus *dpcStatus, pisp_be_global_config &global)
{
	pisp_be_dpc_config dpc = {};

	switch (dpcStatus->strength) {
	case 0: /* "off" */
		break;
	case 1: /* "normal" */
		dpc.coeff_level = 1;
		dpc.coeff_range = 8;
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_DPC;
		break;
	case 2: /* "strong" */
		dpc.coeff_level = 0;
		dpc.coeff_range = 0;
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_DPC;
		break;
	default:
		ASSERT(0);
	}

	be_->SetDpc(dpc);
}

void IpaPiSP::applySdn(const SdnStatus *sdnStatus, pisp_be_global_config &global)
{
	pisp_be_sdn_config sdn = {};
	pisp_bla_config blc;

	be_->GetBlc(blc);
	/* All R/G/B black levels are the same value in the BE after FE alignment */
	sdn.black_level = blc.black_level_r;
	/* leakage is "amount of the original pixel we let through", thus 1 - strength */
	sdn.leakage = clampField(1.0 - sdnStatus->strength, 8, 8);
	sdn.noise_constant = clampField(sdnStatus->noiseConstant, 16);
	sdn.noise_slope = clampField(sdnStatus->noiseSlope, 16, 8);
	sdn.noise_constant2 = clampField(sdnStatus->noiseConstant2, 16);
	sdn.noise_slope2 = clampField(sdnStatus->noiseSlope2, 16, 8);
	be_->SetSdn(sdn);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_SDN;
}

void IpaPiSP::applyTdn(const TdnStatus *tdnStatus, const DeviceStatus *deviceStatus,
		       pisp_be_global_config &global)
{
	utils::Duration exposure = deviceStatus->exposureTime * deviceStatus->analogueGain;
	pisp_be_tdn_config tdn = {};

	double ratio = tdnReset_ ? 1.0 : exposure / lastExposure_;
	if (ratio >= 4.0) {
		/* If the exposure ratio goes above 4x, we need to reset TDN. */
		ratio = 1;
		tdnReset_ = true;
	}

	LOG(IPARPI, Debug) << "TDN: exposure: " << exposure
			   << " last: " << lastExposure_
			   << " ratio: " << ratio;

	pisp_bla_config blc;
	be_->GetBlc(blc);
	/* All R/G/B black levels are the same value in the BE after FE alignment */
	tdn.black_level = blc.black_level_r;
	tdn.ratio = clampField(ratio, 16, 14);
	tdn.noise_constant = clampField(tdnStatus->noiseConstant, 16);
	tdn.noise_slope = clampField(tdnStatus->noiseSlope, 16, 8);
	tdn.threshold = clampField(tdnStatus->threshold, 16, 16);

	global.bayer_enables |= PISP_BE_BAYER_ENABLE_TDN + PISP_BE_BAYER_ENABLE_TDN_OUTPUT;

	/* Only enable the TDN Input after a state reset. */
	if (!tdnReset_) {
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_TDN_INPUT;
		tdn.reset = 0;
	} else
		tdn.reset = 1;

	be_->SetTdn(tdn);
	tdnReset_ = false;
}

void IpaPiSP::applyCdn(const CdnStatus *cdnStatus, pisp_be_global_config &global)
{
	pisp_be_cdn_config cdn = {};

	cdn.thresh = clampField(cdnStatus->threshold, 16);
	cdn.iir_strength = clampField(cdnStatus->strength, 8, 8);
	cdn.g_adjust = clampField(0, 8, 8);
	be_->SetCdn(cdn);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_CDN;
}

void IpaPiSP::applyGeq(const GeqStatus *geqStatus, pisp_be_global_config &global)
{
	pisp_be_geq_config geq = {};

	geq.min = 0;
	geq.max = 0xffff;
	geq.offset = clampField(geqStatus->offset, 16);
	geq.slope_sharper = clampField(geqStatus->slope, 10, 10);
	be_->SetGeq(geq);
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_GEQ;
}

void IpaPiSP::applySaturation(const SaturationStatus *saturationStatus,
			      pisp_be_global_config &global)
{
	pisp_be_sat_control_config saturation;
	pisp_wbg_config wbg;

	saturation.shift_r = std::min<uint8_t>(2, saturationStatus->shiftR);
	saturation.shift_g = std::min<uint8_t>(2, saturationStatus->shiftG);
	saturation.shift_b = std::min<uint8_t>(2, saturationStatus->shiftB);
	be_->SetSatControl(saturation);

	be_->GetWbg(wbg);
	wbg.gain_r >>= saturationStatus->shiftR;
	wbg.gain_g >>= saturationStatus->shiftG;
	wbg.gain_b >>= saturationStatus->shiftB;
	be_->SetWbg(wbg);

	global.rgb_enables |= PISP_BE_RGB_ENABLE_SAT_CONTROL;
}

void IpaPiSP::applySharpen(const SharpenStatus *sharpenStatus,
			   pisp_be_global_config &global)
{
	/*
	 * This threshold scaling is to normalise the VC4 and PiSP parameter
	 * scales in the tuning config.
	 */
	static constexpr double ThresholdScaling = 0.25;
	const double scaling = sharpenStatus->threshold * ThresholdScaling;

	pisp_be_sh_fc_combine_config shfc;
	pisp_be_sharpen_config sharpen;

	be_->InitialiseSharpen(sharpen, shfc);
	sharpen.threshold_offset0 = clampField(sharpen.threshold_offset0 * scaling, 16);
	sharpen.threshold_offset1 = clampField(sharpen.threshold_offset1 * scaling, 16);
	sharpen.threshold_offset2 = clampField(sharpen.threshold_offset2 * scaling, 16);
	sharpen.threshold_offset3 = clampField(sharpen.threshold_offset3 * scaling, 16);
	sharpen.threshold_offset4 = clampField(sharpen.threshold_offset4 * scaling, 16);
	sharpen.threshold_slope0 = clampField(sharpen.threshold_slope0 * scaling, 12);
	sharpen.threshold_slope1 = clampField(sharpen.threshold_slope1 * scaling, 12);
	sharpen.threshold_slope2 = clampField(sharpen.threshold_slope2 * scaling, 12);
	sharpen.threshold_slope3 = clampField(sharpen.threshold_slope3 * scaling, 12);
	sharpen.threshold_slope4 = clampField(sharpen.threshold_slope4 * scaling, 12);
	sharpen.positive_strength = clampField(sharpen.positive_strength * sharpenStatus->strength, 12);
	sharpen.negative_strength = clampField(sharpen.negative_strength * sharpenStatus->strength, 12);
	sharpen.positive_pre_limit = clampField(sharpen.positive_pre_limit * sharpenStatus->limit, 16);
	sharpen.positive_limit = clampField(sharpen.positive_limit * sharpenStatus->limit, 16);
	sharpen.negative_pre_limit = clampField(sharpen.negative_pre_limit * sharpenStatus->limit, 16);
	sharpen.negative_limit = clampField(sharpen.negative_limit * sharpenStatus->limit, 16);

	be_->SetSharpen(sharpen);
	/* The conversion to YCbCr and back is always enabled. */
	global.rgb_enables |= PISP_BE_RGB_ENABLE_SHARPEN;
}

bool IpaPiSP::applyStitch(const StitchStatus *stitchStatus, const DeviceStatus *deviceStatus,
			  const AgcStatus *agcStatus, pisp_be_global_config &global)
{
	/*
	 * Find out what HDR mode/channel this frame is. Normally this will be in the delayed
	 * HDR status (in the AGC status), though after a mode switch this will be absent and
	 * the information will have been stored in the hdrStatus_ field.
	 */
	const HdrStatus *hdrStatus = &hdrStatus_;
	if (agcStatus)
		hdrStatus = &agcStatus->hdr;

	bool modeChange = hdrStatus->mode != lastStitchHdrStatus_.mode;
	bool channelChange = !modeChange && hdrStatus->channel != lastStitchHdrStatus_.channel;
	lastStitchHdrStatus_ = *hdrStatus;

	/* Check for a change of HDR mode. That forces us to start over. */
	if (modeChange)
		lastStitchExposures_.clear();

	if (hdrStatus->channel != "short" && hdrStatus->channel != "long") {
		/* The channel *must* be long or short, anything else does not make sense. */
		LOG(IPARPI, Warning) << "Stitch channel is not long or short";
		return false;
	}

	/* Whatever happens, we're going to output this buffer now. */
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_STITCH_OUTPUT;

	utils::Duration exposure = deviceStatus->exposureTime * deviceStatus->analogueGain;
	lastStitchExposures_[hdrStatus->channel] = exposure;

	/* If the other channel hasn't been seen there's nothing more we can do. */
	std::string otherChannel = hdrStatus->channel == "short" ? "long" : "short";
	if (lastStitchExposures_.find(otherChannel) == lastStitchExposures_.end()) {
		/* The first channel should be "short". */
		if (hdrStatus->channel != "short")
			LOG(IPARPI, Warning) << "First frame is not short";
		return false;
	}

	/* We have both channels, we need to enable stitching. */
	global.bayer_enables |= PISP_BE_BAYER_ENABLE_STITCH_INPUT + PISP_BE_BAYER_ENABLE_STITCH;

	utils::Duration otherExposure = lastStitchExposures_[otherChannel];
	bool phaseLong = hdrStatus->channel == "long";
	double ratio = phaseLong ? otherExposure / exposure : exposure / otherExposure;

	pisp_be_stitch_config stitch = {};
	stitch.exposure_ratio = clampField(ratio, 15, 15);
	if (phaseLong)
		stitch.exposure_ratio |= PISP_BE_STITCH_STREAMING_LONG;
	/* These will be filled in correctly once we have implemented the HDR algorithm. */
	stitch.threshold_lo = stitchStatus->thresholdLo;
	stitch.threshold_diff_power = stitchStatus->diffPower;
	stitch.motion_threshold_256 = stitchStatus->motionThreshold;
	be_->SetStitch(stitch);

	return channelChange;
}

void IpaPiSP::applyTonemap(const TonemapStatus *tonemapStatus, pisp_be_global_config &global)
{
	pisp_be_tonemap_config tonemap = {};

	tonemap.detail_constant = clampField(tonemapStatus->detailConstant, 16);
	tonemap.detail_slope = clampField(tonemapStatus->detailSlope, 16, 8);
	tonemap.iir_strength = clampField(tonemapStatus->iirStrength, 12, 4);
	tonemap.strength = clampField(tonemapStatus->strength, 12, 8);

	if (!generateLut(tonemapStatus->tonemap, tonemap.lut, PISP_BE_TONEMAP_LUT_SIZE)) {
		be_->SetTonemap(tonemap);
		global.bayer_enables |= PISP_BE_BAYER_ENABLE_TONEMAP;
	}
}

void IpaPiSP::applyFocusStats(const NoiseStatus *noiseStatus)
{
	pisp_fe_cdaf_stats_config cdaf;
	fe_->GetCdafStats(cdaf);

	cdaf.noise_constant = noiseStatus->noiseConstant;
	cdaf.noise_slope = noiseStatus->noiseSlope;
	fe_->SetCdafStats(cdaf);
}

void IpaPiSP::applyAF(const struct AfStatus *afStatus, ControlList &lensCtrls)
{
	if (afStatus->lensSetting) {
		ControlValue v(afStatus->lensSetting.value());
		lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE, v);
	}
}

void IpaPiSP::setDefaultConfig()
{
	std::scoped_lock<FrontEnd> l(*fe_);

	pisp_be_global_config beGlobal;
	pisp_fe_global_config feGlobal;

	fe_->GetGlobal(feGlobal);
	be_->GetGlobal(beGlobal);
	/*
	 * Always go to YCbCr and back. We need them if the false colour block is enabled,
	 * and even for mono sensors if sharpening is enabled. So we're better off enabling
	 * them all the time.
	 */
	beGlobal.rgb_enables |= PISP_BE_RGB_ENABLE_YCBCR + PISP_BE_RGB_ENABLE_YCBCR_INVERSE;

	if (!monoSensor()) {
		beGlobal.bayer_enables |= PISP_BE_BAYER_ENABLE_DEMOSAIC;
		beGlobal.rgb_enables |= PISP_BE_RGB_ENABLE_FALSE_COLOUR;
	}

	/*
	 * Ask the AWB algorithm for reasonable gain values so that we can program the
	 * front end stats sensibly. We must also factor in the conversion to luminance.
	 */
	pisp_fe_rgby_config rgby = {};
	double gainR = 1.5, gainB = 1.5;
	RPiController::AwbAlgorithm *awb = dynamic_cast<RPiController::AwbAlgorithm *>(
		controller_.getAlgorithm("awb"));
	if (awb)
		awb->initialValues(gainR, gainB);
	/* The BT.601 RGB -> Y coefficients will do. The precise values are not critical. */
	rgby.gain_r = clampField(gainR * 0.299, 14, 10);
	rgby.gain_g = clampField(1.0 * .587, 14, 10);
	rgby.gain_b = clampField(gainB * .114, 14, 10);
	fe_->SetRGBY(rgby);
	feGlobal.enables |= PISP_FE_ENABLE_RGBY;

	/* Also get sensible front end black level defaults, for the same reason. */
	RPiController::BlackLevelAlgorithm *blackLevel = dynamic_cast<RPiController::BlackLevelAlgorithm *>(
		controller_.getAlgorithm("black_level"));
	if (blackLevel) {
		uint16_t blackLevelR, blackLevelG, blackLevelB;
		BlackLevelStatus blackLevelStatus;

		blackLevel->initialValues(blackLevelR, blackLevelG, blackLevelB);
		blackLevelStatus.blackLevelR = blackLevelR;
		blackLevelStatus.blackLevelG = blackLevelG;
		blackLevelStatus.blackLevelB = blackLevelB;
		applyBlackLevel(&blackLevelStatus, beGlobal);
		feGlobal.enables |= PISP_FE_ENABLE_BLA + PISP_FE_ENABLE_BLC;
	}

	fe_->SetGlobal(feGlobal);
	be_->SetGlobal(beGlobal);
}

void IpaPiSP::setStatsAndDebin()
{
	pisp_fe_crop_config crop{ 0, 0, mode_.width, mode_.height };

	pisp_fe_awb_stats_config awb = {};
	awb.r_lo = awb.g_lo = awb.b_lo = 0;
	awb.r_hi = awb.g_hi = awb.b_hi = 65535 * 0.98;

	pisp_fe_cdaf_stats_config cdaf = {};
	cdaf.mode = (1 << 4) + (1 << 2) + 1; /* Gr / Gb count with weights of (1, 1) */

	{
		std::scoped_lock<FrontEnd> l(*fe_);
		pisp_fe_global_config feGlobal;
		fe_->GetGlobal(feGlobal);
		feGlobal.enables |= PISP_FE_ENABLE_AWB_STATS + PISP_FE_ENABLE_AGC_STATS +
				    PISP_FE_ENABLE_CDAF_STATS;

		fe_->SetGlobal(feGlobal);
		fe_->SetStatsCrop(crop);
		fe_->SetAwbStats(awb);
		fe_->SetCdafStats(cdaf);
	}

	/*
	 * Apply the correct AGC region weights to the Frontend. Need to do this
	 * out of the Frontend scoped lock.
	 */
	setHistogramWeights();

	pisp_be_global_config beGlobal;
	be_->GetGlobal(beGlobal);

	if (mode_.binX > 1 || mode_.binY > 1) {
		pisp_be_debin_config debin;

		be_->GetDebin(debin);
		debin.h_enable = (mode_.binX > 1);
		debin.v_enable = (mode_.binY > 1);
		be_->SetDebin(debin);
		beGlobal.bayer_enables |= PISP_BE_BAYER_ENABLE_DEBIN;
	} else
		beGlobal.bayer_enables &= ~PISP_BE_BAYER_ENABLE_DEBIN;

	be_->SetGlobal(beGlobal);
}

void IpaPiSP::setHistogramWeights()
{
	RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
		controller_.getAlgorithm("agc"));
	if (!agc)
		return;

	const std::vector<double> &weights = agc->getWeights();

	pisp_fe_agc_stats_config config;
	memset(&config, 0, sizeof(config));

	/*
	* The AGC software gives us a 15x15 table of weights which we
	* map onto 16x16 in the hardware, ensuring the rightmost column
	* and bottom row all have zero weight. We align everything to
	* the native 2x2 Bayer pixel blocks.
	*/
	const Size &size = controller_.getHardwareConfig().agcZoneWeights;
	int width = (mode_.width / size.width) & ~1;
	int height = (mode_.height / size.height) & ~1;
	config.offset_x = ((mode_.width - size.width * width) / 2) & ~1;
	config.offset_y = ((mode_.height - size.height * height) / 2) & ~1;
	config.size_x = width;
	config.size_y = height;

	unsigned int idx = 0;
	for (unsigned int row = 0; row < size.height; row++) {
		unsigned int col = 0;
		for (; col < size.width / 2; col++) {
			int wt0 = clampField(weights[idx++], 4, 0, false, "agc weights");
			int wt1 = clampField(weights[idx++], 4, 0, false, "agc weights");
			config.weights[row * 8 + col] = (wt1 << 4) | wt0;
		}
		if (size.width & 1)
			config.weights[row * 8 + col] =
				clampField(weights[idx++], 4, 0, false, "agc weights");
	}

	std::scoped_lock<FrontEnd> l(*fe_);
	fe_->SetAgcStats(config);
}

} /* namespace ipa::RPi */

/*
 * External IPA module interface
 */
extern "C" {
const IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"rpi/pisp",
	"rpi/pisp",
};

IPAInterface *ipaCreate()
{
	return new ipa::RPi::IpaPiSP();
}

} /* extern "C" */

} /* namespace libcamera */
