/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_vimc.cpp - Vimc Image Processing Algorithm module
 */

#include <ipa/ipa_vimc.h>

#include <fcntl.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include <iostream>

#include <ipa/ipa_interface.h>
#include <ipa/ipa_module_info.h>

#include "log.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAVimc)

class IPAVimc : public IPAInterface
{
public:
	IPAVimc();
	~IPAVimc();

	int init() override;
	void configure(const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, ControlInfoMap> &entityControls) override {}
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

int IPAVimc::init()
{
	trace(IPAOperationInit);

	LOG(IPAVimc, Debug) << "initializing vimc IPA!";

	return 0;
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
	"Dummy IPA for Vimc",
	LICENSE,
};

IPAInterface *ipaCreate()
{
	return new IPAVimc();
}
};

}; /* namespace libcamera */
