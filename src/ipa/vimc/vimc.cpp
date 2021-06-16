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

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAVimc)

class IPAVimc : public ipa::vimc::IPAVimcInterface
{
public:
	IPAVimc();
	~IPAVimc();

	int init(const IPASettings &settings) override;

	int start() override;
	void stop() override;

private:
	void initTrace();
	void trace(enum ipa::vimc::IPAOperationCode operation);

	int fd_;
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

int IPAVimc::init(const IPASettings &settings)
{
	trace(ipa::vimc::IPAOperationInit);

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
	trace(ipa::vimc::IPAOperationStart);

	LOG(IPAVimc, Debug) << "start vimc IPA!";

	return 0;
}

void IPAVimc::stop()
{
	trace(ipa::vimc::IPAOperationStop);

	LOG(IPAVimc, Debug) << "stop vimc IPA!";
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
