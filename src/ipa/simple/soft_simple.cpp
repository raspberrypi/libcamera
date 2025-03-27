/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * Simple Software Image Processing Algorithm module
 */

#include <chrono>
#include <stdint.h>
#include <sys/mman.h>

#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/shared_fd.h>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/soft_ipa_interface.h>

#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swisp_stats.h"
#include "libcamera/internal/yaml_parser.h"

#include "libipa/camera_sensor_helper.h"

#include "module.h"

namespace libcamera {
LOG_DEFINE_CATEGORY(IPASoft)

using namespace std::literals::chrono_literals;

namespace ipa::soft {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPASoftSimple : public ipa::soft::IPASoftInterface, public Module
{
public:
	IPASoftSimple()
		: context_(kMaxFrameContexts)
	{
	}

	~IPASoftSimple();

	int init(const IPASettings &settings,
		 const SharedFD &fdStats,
		 const SharedFD &fdParams,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
		 ControlInfoMap *ipaControls,
		 bool *ccmEnabled) override;
	int configure(const IPAConfigInfo &configInfo) override;

	int start() override;
	void stop() override;

	void queueRequest(const uint32_t frame, const ControlList &controls) override;
	void computeParams(const uint32_t frame) override;
	void processStats(const uint32_t frame, const uint32_t bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateExposure(double exposureMSV);

	DebayerParams *params_;
	SwIspStats *stats_;
	std::unique_ptr<CameraSensorHelper> camHelper_;
	ControlInfoMap sensorInfoMap_;

	/* Local parameter storage */
	struct IPAContext context_;
};

IPASoftSimple::~IPASoftSimple()
{
	if (stats_)
		munmap(stats_, sizeof(SwIspStats));
	if (params_)
		munmap(params_, sizeof(DebayerParams));
}

int IPASoftSimple::init(const IPASettings &settings,
			const SharedFD &fdStats,
			const SharedFD &fdParams,
			const IPACameraSensorInfo &sensorInfo,
			const ControlInfoMap &sensorControls,
			ControlInfoMap *ipaControls,
			bool *ccmEnabled)
{
	camHelper_ = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!camHelper_) {
		LOG(IPASoft, Warning)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
	}

	context_.sensorInfo = sensorInfo;

	/* Load the tuning data file */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPASoft, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<libcamera::YamlObject> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	/* \todo Use the IPA configuration file for real. */
	unsigned int version = (*data)["version"].get<uint32_t>(0);
	LOG(IPASoft, Debug) << "Tuning file version " << version;

	if (!data->contains("algorithms")) {
		LOG(IPASoft, Error) << "Tuning file doesn't contain algorithms";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	*ccmEnabled = context_.ccmEnabled;

	params_ = nullptr;
	stats_ = nullptr;

	if (!fdStats.isValid()) {
		LOG(IPASoft, Error) << "Invalid Statistics handle";
		return -ENODEV;
	}

	if (!fdParams.isValid()) {
		LOG(IPASoft, Error) << "Invalid Parameters handle";
		return -ENODEV;
	}

	{
		void *mem = mmap(nullptr, sizeof(DebayerParams), PROT_WRITE,
				 MAP_SHARED, fdParams.get(), 0);
		if (mem == MAP_FAILED) {
			LOG(IPASoft, Error) << "Unable to map Parameters";
			return -errno;
		}

		params_ = static_cast<DebayerParams *>(mem);
	}

	{
		void *mem = mmap(nullptr, sizeof(SwIspStats), PROT_READ,
				 MAP_SHARED, fdStats.get(), 0);
		if (mem == MAP_FAILED) {
			LOG(IPASoft, Error) << "Unable to map Statistics";
			return -errno;
		}

		stats_ = static_cast<SwIspStats *>(mem);
	}

	ControlInfoMap::Map ctrlMap = context_.ctrlMap;
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);

	/*
	 * Check if the sensor driver supports the controls required by the
	 * Soft IPA.
	 * Don't save the min and max control values yet, as e.g. the limits
	 * for V4L2_CID_EXPOSURE depend on the configured sensor resolution.
	 */
	if (sensorControls.find(V4L2_CID_EXPOSURE) == sensorControls.end()) {
		LOG(IPASoft, Error) << "Don't have exposure control";
		return -EINVAL;
	}

	if (sensorControls.find(V4L2_CID_ANALOGUE_GAIN) == sensorControls.end()) {
		LOG(IPASoft, Error) << "Don't have gain control";
		return -EINVAL;
	}

	return 0;
}

int IPASoftSimple::configure(const IPAConfigInfo &configInfo)
{
	sensorInfoMap_ = configInfo.sensorControls;

	const ControlInfo &exposureInfo = sensorInfoMap_.find(V4L2_CID_EXPOSURE)->second;
	const ControlInfo &gainInfo = sensorInfoMap_.find(V4L2_CID_ANALOGUE_GAIN)->second;

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	context_.configuration.agc.lineDuration =
		context_.sensorInfo.minLineLength * 1.0s / context_.sensorInfo.pixelRate;
	context_.configuration.agc.exposureMin = exposureInfo.min().get<int32_t>();
	context_.configuration.agc.exposureMax = exposureInfo.max().get<int32_t>();
	if (!context_.configuration.agc.exposureMin) {
		LOG(IPASoft, Warning) << "Minimum exposure is zero, that can't be linear";
		context_.configuration.agc.exposureMin = 1;
	}

	int32_t againMin = gainInfo.min().get<int32_t>();
	int32_t againMax = gainInfo.max().get<int32_t>();

	if (camHelper_) {
		context_.configuration.agc.againMin = camHelper_->gain(againMin);
		context_.configuration.agc.againMax = camHelper_->gain(againMax);
		context_.configuration.agc.againMinStep =
			(context_.configuration.agc.againMax -
			 context_.configuration.agc.againMin) /
			100.0;
		if (camHelper_->blackLevel().has_value()) {
			/*
			 * The black level from camHelper_ is a 16 bit value, software ISP
			 * works with 8 bit pixel values, both regardless of the actual
			 * sensor pixel width. Hence we obtain the pixel-based black value
			 * by dividing the value from the helper by 256.
			 */
			context_.configuration.black.level =
				camHelper_->blackLevel().value() / 256;
		}
	} else {
		/*
		 * The camera sensor gain (g) is usually not equal to the value written
		 * into the gain register (x). But the way how the AGC algorithm changes
		 * the gain value to make the total exposure closer to the optimum
		 * assumes that g(x) is not too far from linear function. If the minimal
		 * gain is 0, the g(x) is likely to be far from the linear, like
		 * g(x) = a / (b * x + c). To avoid unexpected changes to the gain by
		 * the AGC algorithm (abrupt near one edge, and very small near the
		 * other) we limit the range of the gain values used.
		 */
		context_.configuration.agc.againMax = againMax;
		if (!againMin) {
			LOG(IPASoft, Warning)
				<< "Minimum gain is zero, that can't be linear";
			context_.configuration.agc.againMin =
				std::min(100, againMin / 2 + againMax / 2);
		}
		context_.configuration.agc.againMinStep = 1.0;
	}

	for (auto const &algo : algorithms()) {
		int ret = algo->configure(context_, configInfo);
		if (ret)
			return ret;
	}

	LOG(IPASoft, Info)
		<< "Exposure " << context_.configuration.agc.exposureMin << "-"
		<< context_.configuration.agc.exposureMax
		<< ", gain " << context_.configuration.agc.againMin << "-"
		<< context_.configuration.agc.againMax
		<< " (" << context_.configuration.agc.againMinStep << ")";

	return 0;
}

int IPASoftSimple::start()
{
	return 0;
}

void IPASoftSimple::stop()
{
	context_.frameContexts.clear();
}

void IPASoftSimple::queueRequest(const uint32_t frame, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(frame);

	for (auto const &algo : algorithms())
		algo->queueRequest(context_, frame, frameContext, controls);
}

void IPASoftSimple::computeParams(const uint32_t frame)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);
	for (auto const &algo : algorithms())
		algo->prepare(context_, frame, frameContext, params_);
	setIspParams.emit();
}

void IPASoftSimple::processStats(const uint32_t frame,
				 [[maybe_unused]] const uint32_t bufferId,
				 const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	frameContext.sensor.exposure =
		sensorControls.get(V4L2_CID_EXPOSURE).get<int32_t>();
	int32_t again = sensorControls.get(V4L2_CID_ANALOGUE_GAIN).get<int32_t>();
	frameContext.sensor.gain = camHelper_ ? camHelper_->gain(again) : again;

	ControlList metadata(controls::controls);
	for (auto const &algo : algorithms())
		algo->process(context_, frame, frameContext, stats_, metadata);
	metadataReady.emit(frame, metadata);

	/* Sanity check */
	if (!sensorControls.contains(V4L2_CID_EXPOSURE) ||
	    !sensorControls.contains(V4L2_CID_ANALOGUE_GAIN)) {
		LOG(IPASoft, Error) << "Control(s) missing";
		return;
	}

	ControlList ctrls(sensorInfoMap_);

	auto &againNew = frameContext.sensor.gain;
	ctrls.set(V4L2_CID_EXPOSURE, frameContext.sensor.exposure);
	ctrls.set(V4L2_CID_ANALOGUE_GAIN,
		  static_cast<int32_t>(camHelper_ ? camHelper_->gainCode(againNew) : againNew));

	setSensorControls.emit(ctrls);
}

std::string IPASoftSimple::logPrefix() const
{
	return "IPASoft";
}

} /* namespace ipa::soft */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"simple",
	"simple",
};

IPAInterface *ipaCreate()
{
	return new ipa::soft::IPASoftSimple();
}

} /* extern "C" */

} /* namespace libcamera */
