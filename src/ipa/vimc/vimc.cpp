/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_vimc.cpp - Vimc Image Processing Algorithm module
 */
#include <libcamera/ipa/vimc_ipa_interface.h>

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAVimc)

class IPAVimc : public ipa::vimc::IPAVimcInterface
{
public:
	IPAVimc();
	~IPAVimc();

	int init(const IPASettings &settings,
		 const ipa::vimc::IPAOperationCode code,
		 const Flags<ipa::vimc::TestFlag> inFlags,
		 Flags<ipa::vimc::TestFlag> *outFlags) override;

	int start() override;
	void stop() override;

	int configure(const IPACameraSensorInfo &sensorInfo,
		      const std::map<unsigned int, IPAStream> &streamConfig,
		      const std::map<unsigned int, ControlInfoMap> &entityControls) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void queueRequest(uint32_t frame, const ControlList &controls) override;
	void fillParamsBuffer(uint32_t frame, uint32_t bufferId) override;

private:
	void initTrace();
	void trace(enum ipa::vimc::IPAOperationCode operation);

	int fd_;
	std::map<unsigned int, MappedFrameBuffer> buffers_;
};

IPAVimc::IPAVimc()
	: fd_(-1)
{
	initTrace();
}

IPAVimc::~IPAVimc()
{
	if (fd_ != -1)
		::close(fd_);
}

int IPAVimc::init(const IPASettings &settings,
		  const ipa::vimc::IPAOperationCode code,
		  const Flags<ipa::vimc::TestFlag> inFlags,
		  Flags<ipa::vimc::TestFlag> *outFlags)
{
	trace(ipa::vimc::IPAOperationInit);

	LOG(IPAVimc, Debug)
		<< "initializing vimc IPA with configuration file "
		<< settings.configurationFile;

	LOG(IPAVimc, Debug) << "Got opcode " << code;

	LOG(IPAVimc, Debug)
		<< "Flag 2 was "
		<< (inFlags & ipa::vimc::TestFlag::Flag2 ? "" : "not ")
		<< "set";

	*outFlags |= ipa::vimc::TestFlag::Flag1;

	File conf(settings.configurationFile);
	if (!conf.open(File::OpenModeFlag::ReadOnly)) {
		LOG(IPAVimc, Error) << "Failed to open configuration file";
		return -EINVAL;
	}

	return 0;
}

int IPAVimc::start()
{
	trace(ipa::vimc::IPAOperationStart);

	LOG(IPAVimc, Debug) << "start vimc IPA!";

	return 0;
}

void IPAVimc::stop()
{
	trace(ipa::vimc::IPAOperationStop);

	LOG(IPAVimc, Debug) << "stop vimc IPA!";
}

int IPAVimc::configure([[maybe_unused]] const IPACameraSensorInfo &sensorInfo,
			[[maybe_unused]] const std::map<unsigned int, IPAStream> &streamConfig,
			[[maybe_unused]] const std::map<unsigned int, ControlInfoMap> &entityControls)
{
	LOG(IPAVimc, Debug) << "configure()";

	return 0;
}

void IPAVimc::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		const FrameBuffer fb(buffer.planes);
		buffers_.emplace(std::piecewise_construct,
				 std::forward_as_tuple(buffer.id),
				 std::forward_as_tuple(&fb, MappedFrameBuffer::MapFlag::Read));
	}
}

void IPAVimc::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(it);
	}
}

void IPAVimc::queueRequest([[maybe_unused]] uint32_t frame,
			   [[maybe_unused]] const ControlList &controls)
{
}

void IPAVimc::fillParamsBuffer([[maybe_unused]] uint32_t frame, uint32_t bufferId)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end()) {
		LOG(IPAVimc, Error) << "Could not find parameter buffer";
		return;
	}

	Flags<ipa::vimc::TestFlag> flags;
	paramsBufferReady.emit(bufferId, flags);
}

void IPAVimc::initTrace()
{
	struct stat fifoStat;
	int ret = stat(ipa::vimc::VimcIPAFIFOPath.c_str(), &fifoStat);
	if (ret)
		return;

	ret = ::open(ipa::vimc::VimcIPAFIFOPath.c_str(), O_WRONLY);
	if (ret < 0) {
		ret = errno;
		LOG(IPAVimc, Error) << "Failed to open vimc IPA test FIFO: "
				    << strerror(ret);
		return;
	}

	fd_ = ret;
}

void IPAVimc::trace(enum ipa::vimc::IPAOperationCode operation)
{
	if (fd_ < 0)
		return;

	int ret = ::write(fd_, &operation, sizeof(operation));
	if (ret < 0) {
		ret = errno;
		LOG(IPAVimc, Error) << "Failed to write to vimc IPA test FIFO: "
				    << strerror(ret);
	}
}

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"PipelineHandlerVimc",
	"vimc",
};

IPAInterface *ipaCreate()
{
	return new IPAVimc();
}
}

} /* namespace libcamera */
