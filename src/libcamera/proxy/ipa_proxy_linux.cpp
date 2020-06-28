/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_proxy_linux.cpp - Default Image Processing Algorithm proxy for Linux
 */

#include <vector>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/ipc_unixsocket.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/process.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAProxy)

class IPAProxyLinux : public IPAProxy
{
public:
	IPAProxyLinux(IPAModule *ipam);
	~IPAProxyLinux();

	int init(const IPASettings &settings) override { return 0; }
	int start() override { return 0; }
	void stop() override {}
	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result) override {}
	void mapBuffers(const std::vector<IPABuffer> &buffers) override {}
	void unmapBuffers(const std::vector<unsigned int> &ids) override {}
	void processEvent(const IPAOperationData &event) override {}

private:
	void readyRead(IPCUnixSocket *ipc);

	Process *proc_;

	IPCUnixSocket *socket_;
};

IPAProxyLinux::IPAProxyLinux(IPAModule *ipam)
	: IPAProxy(ipam), proc_(nullptr), socket_(nullptr)
{
	LOG(IPAProxy, Debug)
		<< "initializing dummy proxy: loading IPA from "
		<< ipam->path();

	std::vector<int> fds;
	std::vector<std::string> args;
	args.push_back(ipam->path());
	const std::string path = resolvePath("ipa_proxy_linux");
	if (path.empty()) {
		LOG(IPAProxy, Error)
			<< "Failed to get proxy worker path";
		return;
	}

	socket_ = new IPCUnixSocket();
	int fd = socket_->create();
	if (fd < 0) {
		LOG(IPAProxy, Error)
			<< "Failed to create socket";
		return;
	}
	socket_->readyRead.connect(this, &IPAProxyLinux::readyRead);
	args.push_back(std::to_string(fd));
	fds.push_back(fd);

	proc_ = new Process();
	int ret = proc_->start(path, args, fds);
	if (ret) {
		LOG(IPAProxy, Error)
			<< "Failed to start proxy worker process";
		return;
	}

	valid_ = true;
}

IPAProxyLinux::~IPAProxyLinux()
{
	delete proc_;
	delete socket_;
}

void IPAProxyLinux::readyRead(IPCUnixSocket *ipc)
{
}

REGISTER_IPA_PROXY(IPAProxyLinux)

} /* namespace libcamera */
