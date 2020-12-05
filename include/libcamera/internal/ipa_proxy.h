/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_proxy.h - Image Processing Algorithm proxy
 */
#ifndef __LIBCAMERA_INTERNAL_IPA_PROXY_H__
#define __LIBCAMERA_INTERNAL_IPA_PROXY_H__

#include <memory>
#include <string>
#include <vector>

#include <libcamera/ipa/ipa_interface.h>

namespace libcamera {

class IPAModule;

class IPAProxy : public IPAInterface
{
public:
	IPAProxy(IPAModule *ipam);
	~IPAProxy();

	bool isValid() const { return valid_; }

	std::string configurationFile(const std::string &file) const;

protected:
	std::string resolvePath(const std::string &file) const;

	bool valid_;

private:
	IPAModule *ipam_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_IPA_PROXY_H__ */
