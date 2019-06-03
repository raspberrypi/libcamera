/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_manager.cpp - Image Processing Algorithm module manager
 */

#include "ipa_manager.h"

#include <dirent.h>
#include <string.h>
#include <sys/types.h>

#include "ipa_module.h"
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
	addDir(IPA_MODULE_DIR);

	std::string modulePaths = utils::secure_getenv("LIBCAMERA_IPA_MODULE_PATH");
	if (modulePaths.empty())
		return;

	for (size_t pos = 0, delim = 0; delim != std::string::npos;
	     pos = delim + 1) {
		delim = modulePaths.find(':', pos);
		size_t count = delim == std::string::npos ? delim : delim - pos;
		std::string path = modulePaths.substr(pos, count);
		if (path.empty())
			continue;

		addDir(path.c_str());
	}
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
	if (!dir) {
		int ret = -errno;
		LOG(IPAManager, Error)
			<< "Invalid path " << libDir << " for IPA modules: "
			<< strerror(-ret);
		return ret;
	}

	unsigned int count = 0;
	while ((ent = readdir(dir)) != nullptr) {
		int offset = strlen(ent->d_name) - 3;
		if (offset < 0)
			continue;
		if (strcmp(&ent->d_name[offset], ".so"))
			continue;

		IPAModule *ipaModule = new IPAModule(std::string(libDir) +
						     "/" + ent->d_name);
		if (!ipaModule->isValid()) {
			delete ipaModule;
			continue;
		}

		modules_.push_back(ipaModule);
		count++;
	}

	closedir(dir);
	return count;
}

/**
 * \brief Create an IPA interface that matches a given pipeline handler
 * \param[in] pipe The pipeline handler that wants a matching IPA interface
 * \param[in] minVersion Minimum acceptable version of IPA module
 * \param[in] maxVersion Maximum acceptable version of IPA module
 *
 * \return A newly created IPA interface, or nullptr if no matching
 * IPA module is found
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

	if (!m || !m->load())
		return nullptr;

	return m->createInstance();
}

} /* namespace libcamera */
