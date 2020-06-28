/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_vimc.cpp - Vimc Image Processing Algorithm module
 */

#include <libcamera/ipa/ipa_vimc.h>

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/file.h"
#include "libcamera/internal/log.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAVimc)

class IPAVimc : public IPAInterface
{
public:
	IPAVimc();
	~IPAVimc();

	int init(const IPASettings &settings) override;

	int start() override;
	void stop() override;

	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result) override {}
	void mapBuffers(const std::vector<IPABuffer> &buffers) override {}
	void unmapBuffers(const std::vector<unsigned int> &ids) override {}
	void processEvent(const IPAOperationData &event) override {}

private:
	void initTrace();
	void trace(enum IPAOperationCode operation);

	int fd_;
};

IPAVimc::IPAVimc()
	: fd_(-1)
{
	initTrace();
}

IPAVimc::~IPAVimc()
{
	if (fd_)
		::close(fd_);
}

int IPAVimc::init(const IPASettings &settings)
{
	trace(IPAOperationInit);

	LOG(IPAVimc, Debug)
		<< "initializing vimc IPA with configuration file "
		<< settings.configurationFile;

	File conf(settings.configurationFile);
	if (!conf.open(File::ReadOnly)) {
		LOG(IPAVimc, Error) << "Failed to open configuration file";
		return -EINVAL;
	}

	return 0;
}

int IPAVimc::start()
{
	trace(IPAOperationStart);

	LOG(IPAVimc, Debug) << "start vimc IPA!";

	return 0;
}

void IPAVimc::stop()
{
	trace(IPAOperationStop);

	LOG(IPAVimc, Debug) << "stop vimc IPA!";
}

void IPAVimc::initTrace()
{
	struct stat fifoStat;
	int ret = stat(VIMC_IPA_FIFO_PATH, &fifoStat);
	if (ret)
		return;

	ret = ::open(VIMC_IPA_FIFO_PATH, O_WRONLY);
	if (ret < 0) {
		ret = errno;
		LOG(IPAVimc, Error) << "Failed to open vimc IPA test FIFO: "
				    << strerror(ret);
		return;
	}

	fd_ = ret;
}

void IPAVimc::trace(enum IPAOperationCode operation)
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

struct ipa_context *ipaCreate()
{
	return new IPAInterfaceWrapper(std::make_unique<IPAVimc>());
}
}

} /* namespace libcamera */
