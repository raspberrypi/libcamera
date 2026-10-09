/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * Software ISP Image Processing Algorithm module
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
#include <libcamera/ipa/softisp_ipa_interface.h>

#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swisp_stats.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/adjust.h"
#include "libipa/agc.h"
#include "libipa/camera_sensor_helper.h"

#include "module.h"

namespace libcamera {
LOG_DEFINE_CATEGORY(IPASoftIsp)

using namespace std::literals::chrono_literals;

namespace ipa::softisp {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPASoftIsp : public ipa::softisp::IPASoftIspInterface, public Module
{
public:
	IPASoftIsp()
		: context_(kMaxFrameContexts)
	{
	}

	~IPASoftIsp();

	int init(const IPASettings &settings,
		 const SharedFD &fdStats,
		 const SharedFD &fdParams,
		 const IPACameraSensorInfo &sensorInfo,
		 const ControlInfoMap &sensorControls,
		 ControlInfoMap *ipaControls,
		 bool *ccmEnabled) override;
	int configure(const IPAConfigInfo &configInfo,
		      ControlInfoMap *ipaControls) override;

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

	/* Local parameter storage */
	struct IPAContext context_;
};

IPASoftIsp::~IPASoftIsp()
{
	if (stats_)
		munmap(stats_, sizeof(SwIspStats));
	if (params_)
		munmap(params_, sizeof(DebayerParams));
}

int IPASoftIsp::init(const IPASettings &settings,
		     const SharedFD &fdStats,
		     const SharedFD &fdParams,
		     const IPACameraSensorInfo &sensorInfo,
		     const ControlInfoMap &sensorControls,
		     ControlInfoMap *ipaControls,
		     bool *ccmEnabled)
{
	context_.camHelper = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!context_.camHelper) {
		LOG(IPASoftIsp, Warning)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
	}

	context_.sensorInfo = sensorInfo;
	context_.sensorControls = sensorControls;

	/* Load the tuning data file */
	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPASoftIsp, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<ValueNode> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	/* \todo Use the IPA configuration file for real. */
	unsigned int version = (*data)["version"].get<uint32_t>(0);
	LOG(IPASoftIsp, Debug) << "Tuning file version " << version;

	if (!data->contains("algorithms")) {
		LOG(IPASoftIsp, Error) << "Tuning file doesn't contain algorithms";
		return -EINVAL;
	}

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	*ccmEnabled = context_.ccmEnabled;

	params_ = nullptr;
	stats_ = nullptr;

	if (!fdStats.isValid()) {
		LOG(IPASoftIsp, Error) << "Invalid Statistics handle";
		return -ENODEV;
	}

	if (!fdParams.isValid()) {
		LOG(IPASoftIsp, Error) << "Invalid Parameters handle";
		return -ENODEV;
	}

	{
		void *mem = mmap(nullptr, sizeof(DebayerParams), PROT_WRITE,
				 MAP_SHARED, fdParams.get(), 0);
		if (mem == MAP_FAILED) {
			LOG(IPASoftIsp, Error) << "Unable to map Parameters";
			return -errno;
		}

		params_ = static_cast<DebayerParams *>(mem);
		params_->blackLevel = { { 0.0, 0.0, 0.0 } };
		params_->gamma = 1.0 / algorithms::kDefaultGamma;
		params_->contrastExp = 1.0;
		params_->gains = { { 1.0, 1.0, 1.0 } };
		/* combinedMatrix is reset for each frame. */
	}

	{
		void *mem = mmap(nullptr, sizeof(SwIspStats), PROT_READ,
				 MAP_SHARED, fdStats.get(), 0);
		if (mem == MAP_FAILED) {
			LOG(IPASoftIsp, Error) << "Unable to map Statistics";
			return -errno;
		}

		stats_ = static_cast<SwIspStats *>(mem);
	}

	ControlInfoMap::Map ctrlMap = context_.ctrlMap;
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);

	return 0;
}

int IPASoftIsp::configure(const IPAConfigInfo &configInfo, ControlInfoMap *ipaControls)
{
	context_.sensorControls = configInfo.sensorControls;

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	for (const auto &algo : algorithms()) {
		int ret = algo->configure(context_, configInfo);
		if (ret)
			return ret;
	}

	*ipaControls = { ControlInfoMap::Map(context_.ctrlMap), controls::controls };

	return 0;
}

int IPASoftIsp::start()
{
	return 0;
}

void IPASoftIsp::stop()
{
	context_.frameContexts.clear();
}

void IPASoftIsp::queueRequest(const uint32_t frame, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(frame);

	for (const auto &algo : algorithms())
		algo->queueRequest(context_, frame, frameContext, controls);
}

void IPASoftIsp::computeParams(const uint32_t frame)
{
	context_.activeState.combinedMatrix = Matrix<float, 3, 3>::identity();

	IPAFrameContext &frameContext = context_.frameContexts.get(frame);
	for (const auto &algo : algorithms())
		algo->prepare(context_, frame, frameContext, params_);
	params_->combinedMatrix = context_.activeState.combinedMatrix;

	paramsComputed.emit(frame);
}

void IPASoftIsp::processStats(const uint32_t frame,
			      [[maybe_unused]] const uint32_t bufferId,
			      const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(frame);

	std::tie(frameContext.sensor.exposure, frameContext.sensor.gain) =
		agc::extractControls(sensorControls, context_.camHelper.get());

	ControlList metadata(controls::controls);
	for (const auto &algo : algorithms())
		algo->process(context_, frame, frameContext, stats_, metadata);
	metadataReady.emit(frame, metadata);

	ControlList ctrls(context_.sensorControls);
	agc::prepareControls(ctrls, context_.camHelper.get(),
			     frameContext.agc.exposure, frameContext.agc.gain);
	setSensorControls.emit(ctrls);
}

std::string IPASoftIsp::logPrefix() const
{
	return "IPASoftIsp";
}

} /* namespace ipa::softisp */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"softisp",
};

IPAInterface *ipaCreate()
{
	return new ipa::softisp::IPASoftIsp();
}

} /* extern "C" */

} /* namespace libcamera */
