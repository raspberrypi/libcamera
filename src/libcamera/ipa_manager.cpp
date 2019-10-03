/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_manager.cpp - Image Processing Algorithm module manager
 */

#include "ipa_manager.h"

#include <algorithm>
#include <dirent.h>
#include <string.h>
#include <sys/types.h>

#include "ipa_module.h"
#include "ipa_proxy.h"
#include "log.h"
#include "pipeline_handler.h"
#include "utils.h"

/**
 * \file ipa_manager.h
 * \brief Image Processing Algorithm module manager
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAManager)

/**
 * \class IPAManager
 * \brief Manager for IPA modules
 */

IPAManager::IPAManager()
{
	unsigned int ipaCount = 0;
	int ret;

	ret = addDir(IPA_MODULE_DIR);
	if (ret > 0)
		ipaCount += ret;

	const char *modulePaths = utils::secure_getenv("LIBCAMERA_IPA_MODULE_PATH");
	if (!modulePaths) {
		if (!ipaCount)
			LOG(IPAManager, Warning)
				<< "No IPA found in '" IPA_MODULE_DIR "'";
		return;
	}

	const char *paths = modulePaths;
	while (1) {
		const char *delim = strchrnul(paths, ':');
		size_t count = delim - paths;

		if (count) {
			std::string path(paths, count);
			ret = addDir(path.c_str());
			if (ret > 0)
				ipaCount += ret;
		}

		if (*delim == '\0')
			break;

		paths += count + 1;
	}

	if (!ipaCount)
		LOG(IPAManager, Warning)
			<< "No IPA found in '" IPA_MODULE_DIR "' and '"
			<< modulePaths << "'";
}

IPAManager::~IPAManager()
{
	for (IPAModule *module : modules_)
		delete module;
}

/**
 * \brief Retrieve the IPA manager instance
 *
 * The IPAManager is a singleton and can't be constructed manually. This
 * function shall instead be used to retrieve the single global instance of the
 * manager.
 *
 * \return The IPA manager instance
 */
IPAManager *IPAManager::instance()
{
	static IPAManager ipaManager;
	return &ipaManager;
}

/**
 * \brief Load IPA modules from a directory
 * \param[in] libDir directory to search for IPA modules
 *
 * This method tries to create an IPAModule instance for every shared object
 * found in \a libDir, and skips invalid IPA modules.
 *
 * \return Number of modules loaded by this call, or a negative error code
 * otherwise
 */
int IPAManager::addDir(const char *libDir)
{
	struct dirent *ent;
	DIR *dir;

	dir = opendir(libDir);
	if (!dir)
		return -errno;

	std::vector<std::string> paths;
	while ((ent = readdir(dir)) != nullptr) {
		int offset = strlen(ent->d_name) - 3;
		if (offset < 0)
			continue;
		if (strcmp(&ent->d_name[offset], ".so"))
			continue;

		paths.push_back(std::string(libDir) + "/" + ent->d_name);
	}
	closedir(dir);

	/* Ensure a stable ordering of modules. */
	std::sort(paths.begin(), paths.end());

	unsigned int count = 0;
	for (const std::string &path : paths) {
		IPAModule *ipaModule = new IPAModule(path);
		if (!ipaModule->isValid()) {
			delete ipaModule;
			continue;
		}

		modules_.push_back(ipaModule);
		count++;
	}

	return count;
}

/**
 * \brief Create an IPA interface that matches a given pipeline handler
 * \param[in] pipe The pipeline handler that wants a matching IPA interface
 * \param[in] minVersion Minimum acceptable version of IPA module
 * \param[in] maxVersion Maximum acceptable version of IPA module
 *
 * \return A newly created IPA interface, or nullptr if no matching
 * IPA module is found or if the IPA interface fails to initialize
 */
std::unique_ptr<IPAInterface> IPAManager::createIPA(PipelineHandler *pipe,
						    uint32_t maxVersion,
						    uint32_t minVersion)
{
	IPAModule *m = nullptr;

	for (IPAModule *module : modules_) {
		if (module->match(pipe, minVersion, maxVersion)) {
			m = module;
			break;
		}
	}

	if (!m)
		return nullptr;

	if (!m->isOpenSource()) {
		IPAProxyFactory *pf = nullptr;
		std::vector<IPAProxyFactory *> &factories = IPAProxyFactory::factories();

		for (IPAProxyFactory *factory : factories) {
			/* TODO: Better matching */
			if (!strcmp(factory->name().c_str(), "IPAProxyLinux")) {
				pf = factory;
				break;
			}
		}

		if (!pf) {
			LOG(IPAManager, Error) << "Failed to get proxy factory";
			return nullptr;
		}

		std::unique_ptr<IPAProxy> proxy = pf->create(m);
		if (!proxy->isValid()) {
			LOG(IPAManager, Error) << "Failed to load proxy";
			return nullptr;
		}

		return proxy;
	}

	if (!m->load())
		return nullptr;

	return m->createInstance();
}

} /* namespace libcamera */
