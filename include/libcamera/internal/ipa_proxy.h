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

	void stop() override = 0;

protected:
	std::string resolvePath(const std::string &file) const;

	bool valid_;

private:
	IPAModule *ipam_;
};

class IPAProxyFactory
{
public:
	IPAProxyFactory(const char *name);
	virtual ~IPAProxyFactory() {}

	virtual std::unique_ptr<IPAProxy> create(IPAModule *ipam) = 0;

	const std::string &name() const { return name_; }

	static void registerType(IPAProxyFactory *factory);
	static std::vector<IPAProxyFactory *> &factories();

private:
	std::string name_;
};

#define REGISTER_IPA_PROXY(proxy)			\
class proxy##Factory final : public IPAProxyFactory	\
{							\
public:							\
	proxy##Factory() : IPAProxyFactory(#proxy) {}	\
	std::unique_ptr<IPAProxy> create(IPAModule *ipam)	\
	{						\
		return std::make_unique<proxy>(ipam);	\
	}						\
};							\
static proxy##Factory global_##proxy##Factory;

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_IPA_PROXY_H__ */
