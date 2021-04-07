/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_proxy.cpp - Image Processing Algorithm proxy
 */

#include "libcamera/internal/ipa_proxy.h"

#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/ipa_module.h"

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
 */

/**
 * \enum IPAProxy::ProxyState
 * \brief Identifies the available operational states of the proxy
 *
 * \var IPAProxy::ProxyStopped
 * \brief The proxy is not active and only synchronous operations are permitted
 * \var IPAProxy::ProxyStopping
 * \brief No new tasks can be submitted to the proxy, however existing events
 * can be completed
 * \var IPAProxy::ProxyRunning
 * \brief The Proxy is active and asynchronous tasks may be queued
 */

/**
 * \brief Construct an IPAProxy instance
 * \param[in] ipam The IPA module
 */
IPAProxy::IPAProxy(IPAModule *ipam)
	: valid_(false), state_(ProxyStopped), ipam_(ipam)
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
 * \brief Retrieve the absolute path to an IPA configuration file
 * \param[in] name The configuration file name
 *
 * This function locates the configuration file for an IPA and returns its
 * absolute path. It searches the following directories, in order:
 *
 * - All directories specified in the colon-separated LIBCAMERA_IPA_CONFIG_PATH
 *   environment variable ; or
 * - If libcamera is not installed, the src/ipa/ directory within the source
 *   tree ; otherwise
 * - The system sysconf (etc/libcamera/ipa) and the data (share/libcamera/ipa/)
 *   directories.
 *
 * The system directories are not searched if libcamera is not installed.
 *
 * Within each of those directories, the function looks for a subdirectory
 * named after the IPA module name, as reported in IPAModuleInfo::name, and for
 * a file named \a name within that directory. The \a name is IPA-specific.
 *
 * \return The full path to the IPA configuration file, or an empty string if
 * no configuration file can be found
 */
std::string IPAProxy::configurationFile(const std::string &name) const
{
	struct stat statbuf;
	int ret;

	/*
	 * The IPA module name can be used as-is to build directory names as it
	 * has been validated when loading the module.
	 */
	std::string ipaName = ipam_->info().name;

	/* Check the environment variable first. */
	const char *confPaths = utils::secure_getenv("LIBCAMERA_IPA_CONFIG_PATH");
	if (confPaths) {
		for (const auto &dir : utils::split(confPaths, ":")) {
			if (dir.empty())
				continue;

			std::string confPath = dir + "/" + ipaName + "/" + name;
			ret = stat(confPath.c_str(), &statbuf);
			if (ret == 0 && (statbuf.st_mode & S_IFMT) == S_IFREG)
				return confPath;
		}
	}

	std::string root = utils::libcameraSourcePath();
	if (!root.empty()) {
		/*
		 * When libcamera is used before it is installed, load
		 * configuration files from the source directory. The
		 * configuration files are then located in the 'data'
		 * subdirectory of the corresponding IPA module.
		 */
		std::string ipaConfDir = root + "src/ipa/" + ipaName + "/data";

		LOG(IPAProxy, Info)
			<< "libcamera is not installed. Loading IPA configuration from '"
			<< ipaConfDir << "'";

		std::string confPath = ipaConfDir + "/" + name;
		ret = stat(confPath.c_str(), &statbuf);
		if (ret == 0 && (statbuf.st_mode & S_IFMT) == S_IFREG)
			return confPath;

	} else {
		/* Else look in the system locations. */
		for (const auto &dir : utils::split(IPA_CONFIG_DIR, ":")) {
			std::string confPath = dir + "/" + ipaName + "/" + name;
			ret = stat(confPath.c_str(), &statbuf);
			if (ret == 0 && (statbuf.st_mode & S_IFMT) == S_IFREG)
				return confPath;
		}
	}

	LOG(IPAProxy, Error)
		<< "Configuration file '" << name
		<< "' not found for IPA module '" << ipaName << "'";

	return std::string();
}

/**
 * \brief Find a valid full path for a proxy worker for a given executable name
 * \param[in] file File name of proxy worker executable
 *
 * A proxy worker's executable could be found in either the global installation
 * directory, or in the paths specified by the environment variable
 * LIBCAMERA_IPA_PROXY_PATH. This function checks the global install directory
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
	std::string root = utils::libcameraBuildPath();
	if (!root.empty()) {
		std::string ipaProxyDir = root + "src/libcamera/proxy/worker";

		LOG(IPAProxy, Info)
			<< "libcamera is not installed. Loading proxy workers from '"
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
 * \var IPAProxy::state_
 * \brief Current state of the IPAProxy
 *
 * The IPAProxy can be Running, Stopped, or Stopping.
 *
 * This state provides a means to ensure that asynchronous functions are only
 * called while the proxy is running, and prevent new tasks being submitted
 * while still enabling events to complete when the IPAProxy is stopping.
 */

} /* namespace libcamera */
