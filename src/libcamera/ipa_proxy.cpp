/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_proxy.cpp - Image Processing Algorithm proxy
 */

#include "ipa_proxy.h"

#include <string.h>
#include <unistd.h>

#include "log.h"
#include "utils.h"

#include <iostream>

/**
 * \file ipa_proxy.h
 * \brief IPA Proxy
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAProxy)

/**
 * \class IPAProxy
 * \brief IPA Proxy
 *
 * Isolate IPA into separate process.
 *
 * Every subclass of proxy shall be registered with libcamera using
 * the REGISTER_IPA_PROXY() macro.
 */

/**
 * \brief Construct an IPAProxy instance
 *
 * IPAProxy instances shall be constructed through the IPAProxyFactory::create()
 * method implemented by the respective factories.
 */
IPAProxy::IPAProxy()
	: valid_(false)
{
}

IPAProxy::~IPAProxy()
{
}

/**
 * \fn IPAProxy::isValid()
 * \brief Check if the IPAProxy instance is valid
 *
 * An IPAProxy instance is valid if the IPA interface is successfully created in
 * isolation, and IPC is successfully set up.
 *
 * \return True if the IPAProxy is valid, false otherwise
 */

/**
 * \brief Find a valid full path for a proxy worker for a given executable name
 * \param[in] file File name of proxy worker executable
 *
 * A proxy worker's executable could be found in either the global installation
 * directory, or in the paths specified by the environment variable
 * LIBCAMERA_IPA_PROXY_PATH. This method checks the global install directory
 * first, then LIBCAMERA_IPA_PROXY_PATH in order, and returns the full path to
 * the proxy worker executable that is specified by file. The proxy worker
 * executable shall have exec permission.
 *
 * \return The full path to the proxy worker executable, or an empty string if
 * no valid executable path
 */
std::string IPAProxy::resolvePath(const std::string &file) const
{
	std::string proxyFile = "/" + file;

	/* Check env variable first. */
	const char *execPaths = utils::secure_getenv("LIBCAMERA_IPA_PROXY_PATH");
	if (execPaths) {
		for (const auto &dir : utils::split(execPaths, ":")) {
			if (dir.empty())
				continue;

			std::string proxyPath = dir;
			proxyPath += proxyFile;
			if (!access(proxyPath.c_str(), X_OK))
				return proxyPath;
		}
	}

	/*
	 * When libcamera is used before it is installed, load proxy workers
	 * from the same build directory as the libcamera directory itself.
	 * This requires identifying the path of the libcamera.so, and
	 * referencing a relative path for the proxy workers from that point.
	 */
	if (!utils::isLibcameraInstalled()) {
		std::string ipaProxyDir = utils::dirname(utils::libcameraPath())
					  + "/proxy/worker";

		LOG(IPAProxy, Info)
			<< "libcamera is not installed. Loading proxy workers from'"
			<< ipaProxyDir << "'";

		std::string proxyPath = ipaProxyDir + proxyFile;
		if (!access(proxyPath.c_str(), X_OK))
			return proxyPath;

		return std::string();
	}

	/* Else try finding the exec target from the install directory. */
	std::string proxyPath = std::string(IPA_PROXY_DIR) + proxyFile;
	if (!access(proxyPath.c_str(), X_OK))
		return proxyPath;

	return std::string();
}

/**
 * \var IPAProxy::valid_
 * \brief Flag to indicate if the IPAProxy instance is valid
 *
 * A IPAProxy instance is valid if the IPA interface is successfully created in
 * isolation, and IPC is successfully set up.
 *
 * This flag can be read via IPAProxy::isValid().
 *
 * Implementations of the IPAProxy class should set this flag upon successful
 * construction.
 */

/**
 * \class IPAProxyFactory
 * \brief Registration of IPAProxy classes and creation of instances
 *
 * To facilitate discovery and instantiation of IPAProxy classes, the
 * IPAProxyFactory class maintains a registry of IPAProxy classes. Each
 * IPAProxy subclass shall register itself using the REGISTER_IPA_PROXY()
 * macro, which will create a corresponding instance of a IPAProxyFactory
 * subclass and register it with the static list of factories.
 */

/**
 * \brief Construct a IPAProxy factory
 * \param[in] name Name of the IPAProxy class
 *
 * Creating an instance of the factory registers is with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debugging and IPAProxy matching purposes
 * and shall be unique.
 */
IPAProxyFactory::IPAProxyFactory(const char *name)
	: name_(name)
{
	registerType(this);
}

/**
 * \fn IPAProxyFactory::create()
 * \brief Create an instance of the IPAProxy corresponding to the factory
 * \param[in] ipam The IPA module
 *
 * This virtual function is implemented by the REGISTER_IPA_PROXY() macro.
 * It creates a IPAProxy instance that isolates an IPA interface designated
 * by the IPA module \a ipam.
 *
 * \return A pointer to a newly constructed instance of the IPAProxy subclass
 * corresponding to the factory
 */

/**
 * \fn IPAProxyFactory::name()
 * \brief Retrieve the factory name
 * \return The factory name
 */

/**
 * \brief Add a IPAProxy class to the registry
 * \param[in] factory Factory to use to construct the IPAProxy
 *
 * The caller is responsible to guarantee the uniqueness of the IPAProxy name.
 */
void IPAProxyFactory::registerType(IPAProxyFactory *factory)
{
	std::vector<IPAProxyFactory *> &factories = IPAProxyFactory::factories();

	factories.push_back(factory);

	LOG(IPAProxy, Debug)
		<< "Registered proxy \"" << factory->name() << "\"";
}

/**
 * \brief Retrieve the list of all IPAProxy factories
 *
 * The static factories map is defined inside the function to ensure it gets
 * initialized on first use, without any dependency on link order.
 *
 * \return The list of pipeline handler factories
 */
std::vector<IPAProxyFactory *> &IPAProxyFactory::factories()
{
	static std::vector<IPAProxyFactory *> factories;
	return factories;
}

/**
 * \def REGISTER_IPA_PROXY
 * \brief Register a IPAProxy with the IPAProxy factory
 * \param[in] proxy Class name of IPAProxy derived class to register
 *
 * Register a proxy subclass with the factory and make it available to
 * isolate IPA modules.
 */

} /* namespace libcamera */
