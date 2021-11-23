/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_proxy.h - Image Processing Algorithm proxy
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_interface.h>

namespace libcamera {

class IPAModule;

class IPAProxy : public IPAInterface
{
public:
	enum ProxyState {
		ProxyStopped,
		ProxyStopping,
		ProxyRunning,
	};

	IPAProxy(IPAModule *ipam);
	~IPAProxy();

	bool isValid() const { return valid_; }

	std::string configurationFile(const std::string &file) const;

protected:
	std::string resolvePath(const std::string &file) const;

	bool valid_;
	ProxyState state_;

private:
	IPAModule *ipam_;
};

} /* namespace libcamera */
