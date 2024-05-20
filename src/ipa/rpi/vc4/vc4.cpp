/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi Ltd
 *
 * Raspberry Pi VC4/BCM2835 ISP IPA.
 */

#include <string.h>
#include <sys/mman.h>

#include <linux/bcm2835-isp.h>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "common/ipa_base.h"
#include "controller/af_status.h"
#include "controller/agc_algorithm.h"
#include "controller/alsc_status.h"
#include "controller/awb_status.h"
#include "controller/black_level_status.h"
#include "controller/ccm_status.h"
#include "controller/contrast_status.h"
#include "controller/denoise_algorithm.h"
#include "controller/denoise_status.h"
#include "controller/dpc_status.h"
#include "controller/geq_status.h"
#include "controller/lux_status.h"
#include "controller/noise_status.h"
#include "controller/sharpen_status.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPARPI)

namespace ipa::RPi {

class IpaVc4 final : public IpaBase
{
public:
	IpaVc4()
		: IpaBase(), lsTable_(nullptr)
	{
	}

	~IpaVc4()
	{
		if (lsTable_)
			munmap(lsTable_, MaxLsGridSize);
	}

private:
	int32_t platformInit(const InitParams &params, InitResult *result) override;
	int32_t platformStart(const ControlList &controls, StartResult *result) override;
	int32_t platformConfigure(const ConfigParams &params, ConfigResult *result) override;

	void platformPrepareIsp(const PrepareParams &params, RPiController::Metadata &rpiMetadata) override;
	RPiController::StatisticsPtr platformProcessStats(Span<uint8_t> mem) override;

	void handleControls(const ControlList &controls) override;
	bool validateIspControls();

	void applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls);
	void applyDG(const struct AgcPrepareStatus *dgStatus, ControlList &ctrls);
	void applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls);
	void applyBlackLevel(const struct BlackLevelStatus *blackLevelStatus, ControlList &ctrls);
	void applyGamma(const struct ContrastStatus *contrastStatus, ControlList &ctrls);
	void applyGEQ(const struct GeqStatus *geqStatus, ControlList &ctrls);
	void applyDenoise(const struct DenoiseStatus *denoiseStatus, ControlList &ctrls);
	void applySharpen(const struct SharpenStatus *sharpenStatus, ControlList &ctrls);
	void applyDPC(const struct DpcStatus *dpcStatus, ControlList &ctrls);
	void applyLS(const struct AlscStatus *lsStatus, ControlList &ctrls);
	void applyAF(const struct AfStatus *afStatus, ControlList &lensCtrls);
	void resampleTable(uint16_t dest[], const std::vector<double> &src, int destW, int destH);

	/* VC4 ISP controls. */
	ControlInfoMap ispCtrls_;

	/* LS table allocation passed in from the pipeline handler. */
	SharedFD lsTableHandle_;
	void *lsTable_;
};

int32_t IpaVc4::platformInit([[maybe_unused]] const InitParams &params, [[maybe_unused]] InitResult *result)
{
	const std::string &target = controller_.getTarget();

	if (target != "bcm2835") {
		LOG(IPARPI, Error)
			<< "Tuning data file target returned \"" << target << "\""
			<< ", expected \"bcm2835\"";
		return -EINVAL;
	}

	return 0;
}

int32_t IpaVc4::platformStart([[maybe_unused]] const ControlList &controls,
			      [[maybe_unused]] StartResult *result)
{
	return 0;
}

int32_t IpaVc4::platformConfigure(const ConfigParams &params, [[maybe_unused]] ConfigResult *result)
{
	ispCtrls_ = params.ispControls;
	if (!validateIspControls()) {
		LOG(IPARPI, Error) << "ISP control validation failed.";
		return -1;
	}

	/* Store the lens shading table pointer and handle if available. */
	if (params.lsTableHandle.isValid()) {
		/* Remove any previous table, if there was one. */
		if (lsTable_) {
			munmap(lsTable_, MaxLsGridSize);
			lsTable_ = nullptr;
		}

		/* Map the LS table buffer into user space. */
		lsTableHandle_ = std::move(params.lsTableHandle);
		if (lsTableHandle_.isValid()) {
			lsTable_ = mmap(nullptr, MaxLsGridSize, PROT_READ | PROT_WRITE,
					MAP_SHARED, lsTableHandle_.get(), 0);

			if (lsTable_ == MAP_FAILED) {
				LOG(IPARPI, Error) << "dmaHeap mmap failure for LS table.";
				lsTable_ = nullptr;
			}
		}
	}

	return 0;
}

void IpaVc4::platformPrepareIsp([[maybe_unused]] const PrepareParams &params,
				RPiController::Metadata &rpiMetadata)
{
	ControlList ctrls(ispCtrls_);

	/* Lock the metadata buffer to avoid constant locks/unlocks. */
	std::unique_lock<RPiController::Metadata> lock(rpiMetadata);

	AwbStatus *awbStatus = rpiMetadata.getLocked<AwbStatus>("awb.status");
	if (awbStatus)
		applyAWB(awbStatus, ctrls);

	CcmStatus *ccmStatus = rpiMetadata.getLocked<CcmStatus>("ccm.status");
	if (ccmStatus)
		applyCCM(ccmStatus, ctrls);

	AgcPrepareStatus *dgStatus = rpiMetadata.getLocked<AgcPrepareStatus>("agc.prepare_status");
	if (dgStatus)
		applyDG(dgStatus, ctrls);

	AlscStatus *lsStatus = rpiMetadata.getLocked<AlscStatus>("alsc.status");
	if (lsStatus)
		applyLS(lsStatus, ctrls);

	ContrastStatus *contrastStatus = rpiMetadata.getLocked<ContrastStatus>("contrast.status");
	if (contrastStatus)
		applyGamma(contrastStatus, ctrls);

	BlackLevelStatus *blackLevelStatus = rpiMetadata.getLocked<BlackLevelStatus>("black_level.status");
	if (blackLevelStatus)
		applyBlackLevel(blackLevelStatus, ctrls);

	GeqStatus *geqStatus = rpiMetadata.getLocked<GeqStatus>("geq.status");
	if (geqStatus)
		applyGEQ(geqStatus, ctrls);

	DenoiseStatus *denoiseStatus = rpiMetadata.getLocked<DenoiseStatus>("denoise.status");
	if (denoiseStatus)
		applyDenoise(denoiseStatus, ctrls);

	SharpenStatus *sharpenStatus = rpiMetadata.getLocked<SharpenStatus>("sharpen.status");
	if (sharpenStatus)
		applySharpen(sharpenStatus, ctrls);

	DpcStatus *dpcStatus = rpiMetadata.getLocked<DpcStatus>("dpc.status");
	if (dpcStatus)
		applyDPC(dpcStatus, ctrls);

	const AfStatus *afStatus = rpiMetadata.getLocked<AfStatus>("af.status");
	if (afStatus) {
		ControlList lensctrls(lensCtrls_);
		applyAF(afStatus, lensctrls);
		if (!lensctrls.empty())
			setLensControls.emit(lensctrls);
	}

	if (!ctrls.empty())
		setIspControls.emit(ctrls);
}

RPiController::StatisticsPtr IpaVc4::platformProcessStats(Span<uint8_t> mem)
{
	using namespace RPiController;

	const bcm2835_isp_stats *stats = reinterpret_cast<bcm2835_isp_stats *>(mem.data());
	StatisticsPtr statistics = std::make_shared<Statistics>(Statistics::AgcStatsPos::PreWb,
								Statistics::ColourStatsPos::PostLsc);
	const Controller::HardwareConfig &hw = controller_.getHardwareConfig();
	unsigned int i;

	/* RGB histograms are not used, so do not populate them. */
	statistics->yHist = RPiController::Histogram(stats->hist[0].g_hist,
						     hw.numHistogramBins);

	/* All region sums are based on a 16-bit normalised pipeline bit-depth. */
	unsigned int scale = Statistics::NormalisationFactorPow2 - hw.pipelineWidth;

	statistics->awbRegions.init(hw.awbRegions);
	for (i = 0; i < statistics->awbRegions.numRegions(); i++)
		statistics->awbRegions.set(i, { { stats->awb_stats[i].r_sum << scale,
						  stats->awb_stats[i].g_sum << scale,
						  stats->awb_stats[i].b_sum << scale },
						stats->awb_stats[i].counted,
						stats->awb_stats[i].notcounted });

	RPiController::AgcAlgorithm *agc = dynamic_cast<RPiController::AgcAlgorithm *>(
		controller_.getAlgorithm("agc"));
	if (!agc) {
		LOG(IPARPI, Debug) << "No AGC algorithm - not copying statistics";
		statistics->agcRegions.init(0);
	} else {
		statistics->agcRegions.init(hw.agcRegions);
		const std::vector<double> &weights = agc->getWeights();
		for (i = 0; i < statistics->agcRegions.numRegions(); i++) {
			uint64_t rSum = (stats->agc_stats[i].r_sum << scale) * weights[i];
			uint64_t gSum = (stats->agc_stats[i].g_sum << scale) * weights[i];
			uint64_t bSum = (stats->agc_stats[i].b_sum << scale) * weights[i];
			uint32_t counted = stats->agc_stats[i].counted * weights[i];
			uint32_t notcounted = stats->agc_stats[i].notcounted * weights[i];
			statistics->agcRegions.set(i, { { rSum, gSum, bSum },
							counted,
							notcounted });
		}
	}

	statistics->focusRegions.init(hw.focusRegions);
	for (i = 0; i < statistics->focusRegions.numRegions(); i++)
		statistics->focusRegions.set(i, { stats->focus_stats[i].contrast_val[1][1] / 1000,
						  stats->focus_stats[i].contrast_val_num[1][1],
						  stats->focus_stats[i].contrast_val_num[1][0] });

	if (statsMetadataOutput_) {
		Span<const uint8_t> statsSpan(reinterpret_cast<const uint8_t *>(stats),
					      sizeof(bcm2835_isp_stats));
		libcameraMetadata_.set(controls::rpi::Bcm2835StatsOutput, statsSpan);
	}

	return statistics;
}

void IpaVc4::handleControls(const ControlList &controls)
{
	static const std::map<int32_t, RPiController::DenoiseMode> DenoiseModeTable = {
		{ controls::draft::NoiseReductionModeOff, RPiController::DenoiseMode::Off },
		{ controls::draft::NoiseReductionModeFast, RPiController::DenoiseMode::ColourFast },
		{ controls::draft::NoiseReductionModeHighQuality, RPiController::DenoiseMode::ColourHighQuality },
		{ controls::draft::NoiseReductionModeMinimal, RPiController::DenoiseMode::ColourOff },
		{ controls::draft::NoiseReductionModeZSL, RPiController::DenoiseMode::ColourHighQuality },
	};

	for (auto const &ctrl : controls) {
		switch (ctrl.first) {
		case controls::draft::NOISE_REDUCTION_MODE: {
			RPiController::DenoiseAlgorithm *sdn = dynamic_cast<RPiController::DenoiseAlgorithm *>(
				controller_.getAlgorithm("SDN"));
			/* Some platforms may have a combined "denoise" algorithm instead. */
			if (!sdn)
				sdn = dynamic_cast<RPiController::DenoiseAlgorithm *>(
					controller_.getAlgorithm("denoise"));
			if (!sdn) {
				LOG(IPARPI, Warning)
					<< "Could not set NOISE_REDUCTION_MODE - no SDN algorithm";
				return;
			}

			int32_t idx = ctrl.second.get<int32_t>();
			auto mode = DenoiseModeTable.find(idx);
			if (mode != DenoiseModeTable.end())
				sdn->setMode(mode->second);
			break;
		}
		}
	}
}

bool IpaVc4::validateIspControls()
{
	static const uint32_t ctrls[] = {
		V4L2_CID_RED_BALANCE,
		V4L2_CID_BLUE_BALANCE,
		V4L2_CID_DIGITAL_GAIN,
		V4L2_CID_USER_BCM2835_ISP_CC_MATRIX,
		V4L2_CID_USER_BCM2835_ISP_GAMMA,
		V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL,
		V4L2_CID_USER_BCM2835_ISP_GEQ,
		V4L2_CID_USER_BCM2835_ISP_DENOISE,
		V4L2_CID_USER_BCM2835_ISP_SHARPEN,
		V4L2_CID_USER_BCM2835_ISP_DPC,
		V4L2_CID_USER_BCM2835_ISP_LENS_SHADING,
		V4L2_CID_USER_BCM2835_ISP_CDN,
	};

	for (auto c : ctrls) {
		if (ispCtrls_.find(c) == ispCtrls_.end()) {
			LOG(IPARPI, Error) << "Unable to find ISP control "
					   << utils::hex(c);
			return false;
		}
	}

	return true;
}

void IpaVc4::applyAWB(const struct AwbStatus *awbStatus, ControlList &ctrls)
{
	LOG(IPARPI, Debug) << "Applying WB R: " << awbStatus->gainR << " B: "
			   << awbStatus->gainB;

	ctrls.set(V4L2_CID_RED_BALANCE,
		  static_cast<int32_t>(awbStatus->gainR * 1000));
	ctrls.set(V4L2_CID_BLUE_BALANCE,
		  static_cast<int32_t>(awbStatus->gainB * 1000));
}

void IpaVc4::applyDG(const struct AgcPrepareStatus *dgStatus, ControlList &ctrls)
{
	ctrls.set(V4L2_CID_DIGITAL_GAIN,
		  static_cast<int32_t>(dgStatus->digitalGain * 1000));
}

void IpaVc4::applyCCM(const struct CcmStatus *ccmStatus, ControlList &ctrls)
{
	bcm2835_isp_custom_ccm ccm;

	for (int i = 0; i < 9; i++) {
		ccm.ccm.ccm[i / 3][i % 3].den = 1000;
		ccm.ccm.ccm[i / 3][i % 3].num = 1000 * ccmStatus->matrix[i];
	}

	ccm.enabled = 1;
	ccm.ccm.offsets[0] = ccm.ccm.offsets[1] = ccm.ccm.offsets[2] = 0;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&ccm),
					    sizeof(ccm) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_CC_MATRIX, c);
}

void IpaVc4::applyBlackLevel(const struct BlackLevelStatus *blackLevelStatus, ControlList &ctrls)
{
	bcm2835_isp_black_level blackLevel;

	blackLevel.enabled = 1;
	blackLevel.black_level_r = blackLevelStatus->blackLevelR;
	blackLevel.black_level_g = blackLevelStatus->blackLevelG;
	blackLevel.black_level_b = blackLevelStatus->blackLevelB;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&blackLevel),
					    sizeof(blackLevel) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_BLACK_LEVEL, c);
}

void IpaVc4::applyGamma(const struct ContrastStatus *contrastStatus, ControlList &ctrls)
{
	const unsigned int numGammaPoints = controller_.getHardwareConfig().numGammaPoints;
	struct bcm2835_isp_gamma gamma;

	for (unsigned int i = 0; i < numGammaPoints - 1; i++) {
		int x = i < 16 ? i * 1024
			       : (i < 24 ? (i - 16) * 2048 + 16384
					 : (i - 24) * 4096 + 32768);
		gamma.x[i] = x;
		gamma.y[i] = std::min<uint16_t>(65535, contrastStatus->gammaCurve.eval(x));
	}

	gamma.x[numGammaPoints - 1] = 65535;
	gamma.y[numGammaPoints - 1] = 65535;
	gamma.enabled = 1;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&gamma),
					    sizeof(gamma) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_GAMMA, c);
}

void IpaVc4::applyGEQ(const struct GeqStatus *geqStatus, ControlList &ctrls)
{
	bcm2835_isp_geq geq;

	geq.enabled = 1;
	geq.offset = geqStatus->offset;
	geq.slope.den = 1000;
	geq.slope.num = 1000 * geqStatus->slope;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&geq),
					    sizeof(geq) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_GEQ, c);
}

void IpaVc4::applyDenoise(const struct DenoiseStatus *denoiseStatus, ControlList &ctrls)
{
	using RPiController::DenoiseMode;

	bcm2835_isp_denoise denoise;
	DenoiseMode mode = static_cast<DenoiseMode>(denoiseStatus->mode);

	denoise.enabled = mode != DenoiseMode::Off;
	denoise.constant = denoiseStatus->noiseConstant;
	denoise.slope.num = 1000 * denoiseStatus->noiseSlope;
	denoise.slope.den = 1000;
	denoise.strength.num = 1000 * denoiseStatus->strength;
	denoise.strength.den = 1000;

	/* Set the CDN mode to match the SDN operating mode. */
	bcm2835_isp_cdn cdn;
	switch (mode) {
	case DenoiseMode::ColourFast:
		cdn.enabled = 1;
		cdn.mode = CDN_MODE_FAST;
		break;
	case DenoiseMode::ColourHighQuality:
		cdn.enabled = 1;
		cdn.mode = CDN_MODE_HIGH_QUALITY;
		break;
	default:
		cdn.enabled = 0;
	}

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&denoise),
					    sizeof(denoise) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_DENOISE, c);

	c = ControlValue(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&cdn),
					      sizeof(cdn) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_CDN, c);
}

void IpaVc4::applySharpen(const struct SharpenStatus *sharpenStatus, ControlList &ctrls)
{
	bcm2835_isp_sharpen sharpen;

	sharpen.enabled = 1;
	sharpen.threshold.num = 1000 * sharpenStatus->threshold;
	sharpen.threshold.den = 1000;
	sharpen.strength.num = 1000 * sharpenStatus->strength;
	sharpen.strength.den = 1000;
	sharpen.limit.num = 1000 * sharpenStatus->limit;
	sharpen.limit.den = 1000;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&sharpen),
					    sizeof(sharpen) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_SHARPEN, c);
}

void IpaVc4::applyDPC(const struct DpcStatus *dpcStatus, ControlList &ctrls)
{
	bcm2835_isp_dpc dpc;

	dpc.enabled = 1;
	dpc.strength = dpcStatus->strength;

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&dpc),
					    sizeof(dpc) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_DPC, c);
}

void IpaVc4::applyLS(const struct AlscStatus *lsStatus, ControlList &ctrls)
{
	/*
	 * Program lens shading tables into pipeline.
	 * Choose smallest cell size that won't exceed 63x48 cells.
	 */
	const int cellSizes[] = { 16, 32, 64, 128, 256 };
	unsigned int numCells = std::size(cellSizes);
	unsigned int i, w, h, cellSize;
	for (i = 0; i < numCells; i++) {
		cellSize = cellSizes[i];
		w = (mode_.width + cellSize - 1) / cellSize;
		h = (mode_.height + cellSize - 1) / cellSize;
		if (w < 64 && h <= 48)
			break;
	}

	if (i == numCells) {
		LOG(IPARPI, Error) << "Cannot find cell size";
		return;
	}

	/* We're going to supply corner sampled tables, 16 bit samples. */
	w++, h++;
	bcm2835_isp_lens_shading ls = {
		.enabled = 1,
		.grid_cell_size = cellSize,
		.grid_width = w,
		.grid_stride = w,
		.grid_height = h,
		/* .dmabuf will be filled in by pipeline handler. */
		.dmabuf = 0,
		.ref_transform = 0,
		.corner_sampled = 1,
		.gain_format = GAIN_FORMAT_U4P10
	};

	if (!lsTable_ || w * h * 4 * sizeof(uint16_t) > MaxLsGridSize) {
		LOG(IPARPI, Error) << "Do not have a correctly allocate lens shading table!";
		return;
	}

	if (lsStatus) {
		/* Format will be u4.10 */
		uint16_t *grid = static_cast<uint16_t *>(lsTable_);

		resampleTable(grid, lsStatus->r, w, h);
		resampleTable(grid + w * h, lsStatus->g, w, h);
		memcpy(grid + 2 * w * h, grid + w * h, w * h * sizeof(uint16_t));
		resampleTable(grid + 3 * w * h, lsStatus->b, w, h);
	}

	ControlValue c(Span<const uint8_t>{ reinterpret_cast<uint8_t *>(&ls),
					    sizeof(ls) });
	ctrls.set(V4L2_CID_USER_BCM2835_ISP_LENS_SHADING, c);
}

void IpaVc4::applyAF(const struct AfStatus *afStatus, ControlList &lensCtrls)
{
	if (afStatus->lensSetting) {
		ControlValue v(afStatus->lensSetting.value());
		lensCtrls.set(V4L2_CID_FOCUS_ABSOLUTE, v);
	}
}

/*
 * Resamples a 16x12 table with central sampling to destW x destH with corner
 * sampling.
 */
void IpaVc4::resampleTable(uint16_t dest[], const std::vector<double> &src,
			   int destW, int destH)
{
	/*
	 * Precalculate and cache the x sampling locations and phases to
	 * save recomputing them on every row.
	 */
	assert(destW > 1 && destH > 1 && destW <= 64);
	int xLo[64], xHi[64];
	double xf[64];
	double x = -0.5, xInc = 16.0 / (destW - 1);
	for (int i = 0; i < destW; i++, x += xInc) {
		xLo[i] = floor(x);
		xf[i] = x - xLo[i];
		xHi[i] = xLo[i] < 15 ? xLo[i] + 1 : 15;
		xLo[i] = xLo[i] > 0 ? xLo[i] : 0;
	}

	/* Now march over the output table generating the new values. */
	double y = -0.5, yInc = 12.0 / (destH - 1);
	for (int j = 0; j < destH; j++, y += yInc) {
		int yLo = floor(y);
		double yf = y - yLo;
		int yHi = yLo < 11 ? yLo + 1 : 11;
		yLo = yLo > 0 ? yLo : 0;
		double const *rowAbove = src.data() + yLo * 16;
		double const *rowBelow = src.data() + yHi * 16;
		for (int i = 0; i < destW; i++) {
			double above = rowAbove[xLo[i]] * (1 - xf[i]) + rowAbove[xHi[i]] * xf[i];
			double below = rowBelow[xLo[i]] * (1 - xf[i]) + rowBelow[xHi[i]] * xf[i];
			int result = floor(1024 * (above * (1 - yf) + below * yf) + .5);
			*(dest++) = result > 16383 ? 16383 : result; /* want u4.10 */
		}
	}
}

} /* namespace ipa::RPi */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"rpi/vc4",
	"rpi/vc4",
};

IPAInterface *ipaCreate()
{
	return new ipa::RPi::IpaVc4();
}

} /* extern "C" */

} /* namespace libcamera */
