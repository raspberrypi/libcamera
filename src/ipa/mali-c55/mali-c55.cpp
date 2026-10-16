/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Ideas on Board Oy
 *
 * Mali-C55 ISP image processing algorithms
 */

#include <map>
#include <span>
#include <string.h>
#include <vector>

#include <linux/media/arm/mali-c55-config.h>
#include <linux/v4l2-controls.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>
#include <libcamera/ipa/mali-c55_ipa_interface.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/yaml_parser.h"

#include "algorithms/algorithm.h"
#include "libipa/agc.h"
#include "libipa/camera_sensor_helper.h"

#include "ipa_context.h"
#include "params.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAMaliC55)

using namespace std::literals::chrono_literals;

namespace ipa::mali_c55 {

/* Maximum number of frame contexts to be held */
static constexpr uint32_t kMaxFrameContexts = 16;

class IPAMaliC55 : public IPAMaliC55Interface, public Module
{
public:
	IPAMaliC55();

	int init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		 ControlInfoMap *ipaControls) override;
	int start() override;
	void stop() override;
	int configure(const IPAConfigInfo &ipaConfig, uint8_t bayerOrder,
		      ControlInfoMap *ipaControls) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers, bool readOnly) override;
	void unmapBuffers(const std::vector<IPABuffer> &buffers) override;
	void queueRequest(const uint32_t request, const ControlList &controls) override;
	void fillParams(unsigned int request, uint32_t bufferId) override;
	void processStats(unsigned int request, unsigned int bufferId,
			  const ControlList &sensorControls) override;

protected:
	std::string logPrefix() const override;

private:
	void updateControls(ControlInfoMap *ipaControls);
	void setControls(const IPAFrameContext &frameContext);

	std::map<unsigned int, MappedFrameBuffer> buffers_;

	/* Local parameter storage */
	struct IPAContext context_;
};

namespace {

} /* namespace */

IPAMaliC55::IPAMaliC55()
	: context_(kMaxFrameContexts)
{
}

std::string IPAMaliC55::logPrefix() const
{
	return "mali-c55";
}

int IPAMaliC55::init(const IPASettings &settings, const IPAConfigInfo &ipaConfig,
		     ControlInfoMap *ipaControls)
{
	context_.sensorInfo = ipaConfig.sensorInfo;

	context_.camHelper = CameraSensorHelperFactoryBase::create(settings.sensorModel);
	if (!context_.camHelper) {
		LOG(IPAMaliC55, Error)
			<< "Failed to create camera sensor helper for "
			<< settings.sensorModel;
		return -ENODEV;
	}

	File file(settings.configurationFile);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		int ret = file.error();
		LOG(IPAMaliC55, Error)
			<< "Failed to open configuration file "
			<< settings.configurationFile << ": " << strerror(-ret);
		return ret;
	}

	std::unique_ptr<ValueNode> data = YamlParser::parse(file);
	if (!data)
		return -EINVAL;

	if (!data->contains("algorithms")) {
		LOG(IPAMaliC55, Error)
			<< "Tuning file doesn't contain any algorithm";
		return -EINVAL;
	}

	context_.sensorControls = ipaConfig.sensorControls;

	int ret = createAlgorithms(context_, (*data)["algorithms"]);
	if (ret)
		return ret;

	updateControls(ipaControls);

	return 0;
}

void IPAMaliC55::setControls(const IPAFrameContext &frameContext)
{
	ControlList ctrls(context_.sensorControls);
	agc::prepareControls(ctrls, context_.camHelper.get(),
			     frameContext.agc.exposure, frameContext.agc.gain);

	setSensorControls.emit(ctrls);
}

int IPAMaliC55::start()
{
	return 0;
}

void IPAMaliC55::stop()
{
	context_.frameContexts.clear();
}

void IPAMaliC55::updateControls(ControlInfoMap *ipaControls)
{
	ControlInfoMap::Map ctrlMap;

	ctrlMap.insert(context_.ctrlMap.begin(), context_.ctrlMap.end());
	*ipaControls = ControlInfoMap(std::move(ctrlMap), controls::controls);
}

int IPAMaliC55::configure(const IPAConfigInfo &ipaConfig, uint8_t bayerOrder,
			  ControlInfoMap *ipaControls)
{
	context_.sensorControls = ipaConfig.sensorControls;
	context_.sensorInfo = ipaConfig.sensorInfo;

	/* Clear the IPA context before the streaming session. */
	context_.configuration = {};
	context_.activeState = {};
	context_.frameContexts.clear();

	const IPACameraSensorInfo &info = ipaConfig.sensorInfo;

	context_.configuration.sensor.bayerOrder = static_cast<BayerFormat::Order>(bayerOrder);

	if (auto bl = context_.camHelper->blackLevel()) {
		/*
		 * The black level from CameraSensorHelper is a 16-bit value.
		 * The Mali-C55 ISP expects 20-bit settings, so we shift it to
		 * the appropriate width
		 */
		context_.configuration.sensor.blackLevel = *bl << 4;
	}

	for (const auto &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		int ret = algo->configure(context_, info);
		if (ret)
			return ret;
	}

	updateControls(ipaControls);

	return 0;
}

void IPAMaliC55::mapBuffers(const std::vector<IPABuffer> &buffers, bool readOnly)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(
			buffer.id,
			MappedFrameBuffer(
				&fb,
				readOnly ? MappedFrameBuffer::MapFlag::Read
					 : MappedFrameBuffer::MapFlag::ReadWrite));
	}
}

void IPAMaliC55::unmapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		auto it = buffers_.find(buffer.id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(buffer.id);
	}
}

void IPAMaliC55::queueRequest(const uint32_t request, const ControlList &controls)
{
	IPAFrameContext &frameContext = context_.frameContexts.alloc(request);

	for (const auto &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->queueRequest(context_, request, frameContext, controls);
	}
}

void IPAMaliC55::fillParams(unsigned int request,
			    [[maybe_unused]] uint32_t bufferId)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(request);
	MaliC55Params params(buffers_.at(bufferId).planes()[0]);

	for (const auto &algo : algorithms())
		algo->prepare(context_, request, frameContext, &params);

	paramsComputed.emit(request, params.bytesused());
}

void IPAMaliC55::processStats(unsigned int request, unsigned int bufferId,
			      const ControlList &sensorControls)
{
	IPAFrameContext &frameContext = context_.frameContexts.get(request);
	const mali_c55_stats_buffer *stats = nullptr;

	stats = reinterpret_cast<mali_c55_stats_buffer *>(
		buffers_.at(bufferId).planes()[0].data());

	std::tie(frameContext.sensor.exposure, frameContext.sensor.gain) =
		agc::extractControls(sensorControls, context_.camHelper.get());

	ControlList metadata(controls::controls);

	for (const auto &a : algorithms()) {
		Algorithm *algo = static_cast<Algorithm *>(a.get());

		algo->process(context_, request, frameContext, stats, metadata);
	}

	setControls(frameContext);

	statsProcessed.emit(request, metadata);
}

} /* namespace ipa::mali_c55 */

/*
 * External IPA module interface
 */
extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	1,
	"mali-c55",
};

IPAInterface *ipaCreate()
{
	return new ipa::mali_c55::IPAMaliC55();
}

} /* extern "C" */

} /* namespace libcamera */
