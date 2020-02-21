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

#include "ipa_context_wrapper.h"
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
 *
 * The IPA module manager discovers IPA modules from disk, queries and loads
 * them, and creates IPA contexts. It supports isolation of the modules in a
 * separate process with IPC communication and offers a unified IPAInterface
 * view of the IPA contexts to pipeline handlers regardless of whether the
 * modules are isolated or loaded in the same process.
 *
 * Module isolation is based on the module licence. Open-source modules are
 * loaded without isolation, while closed-source module are forcefully isolated.
 * The isolation mechanism ensures that no code from a closed-source module is
 * ever run in the libcamera process.
 *
 * To create an IPA context, pipeline handlers call the IPAManager::ipaCreate()
 * method. For a directly loaded module, the manager calls the module's
 * ipaCreate() function directly and wraps the returned context in an
 * IPAContextWrapper that exposes an IPAInterface.
 *
 * ~~~~
 * +---------------+
 * |   Pipeline    |
 * |    Handler    |
 * +---------------+
 *         |
 *         v
 * +---------------+                   +---------------+
 * |      IPA      |                   |  Open Source  |
 * |   Interface   |                   |  IPA Module   |
 * | - - - - - - - |                   | - - - - - - - |
 * |  IPA Context  |  ipa_context_ops  |  ipa_context  |
 * |    Wrapper    | ----------------> |               |
 * +---------------+                   +---------------+
 * ~~~~
 *
 * For an isolated module, the manager instantiates an IPAProxy which spawns a
 * new process for an IPA proxy worker. The worker loads the IPA module and
 * creates the IPA context. The IPAProxy alse exposes an IPAInterface.
 *
 * ~~~~
 * +---------------+                   +---------------+
 * |   Pipeline    |                   | Closed Source |
 * |    Handler    |                   |  IPA Module   |
 * +---------------+                   | - - - - - - - |
 *         |                           |  ipa_context  |
 *         v                           |               |
 * +---------------+                   +---------------+
 * |      IPA      |           ipa_context_ops ^
 * |   Interface   |                           |
 * | - - - - - - - |                   +---------------+
 * |   IPA Proxy   |     operations    |   IPA Proxy   |
 * |               | ----------------> |    Worker     |
 * +---------------+      over IPC     +---------------+
 * ~~~~
 *
 * The IPAInterface implemented by the IPAContextWrapper or IPAProxy is
 * returned to the pipeline handler, and all interactions with the IPA context
 * go the same interface regardless of process isolation.
 *
 * In all cases the data passed to the IPAInterface methods is serialized to
 * Plain Old Data, either for the purpose of passing it to the IPA context
 * plain C API, or to transmit the data to the isolated process through IPC.
 */

IPAManager::IPAManager()
{
	unsigned int ipaCount = 0;

	/* User-specified paths take precedence. */
	const char *modulePaths = utils::secure_getenv("LIBCAMERA_IPA_MODULE_PATH");
	if (modulePaths) {
		for (const auto &dir : utils::split(modulePaths, ":")) {
			if (dir.empty())
				continue;

			ipaCount += addDir(dir.c_str());
		}

		if (!ipaCount)
			LOG(IPAManager, Warning)
				<< "No IPA found in '" << modulePaths << "'";
	}

	/* Load IPAs from the installed system path. */
	ipaCount += addDir(IPA_MODULE_DIR);

	if (!ipaCount)
		LOG(IPAManager, Warning)
			<< "No IPA found in '" IPA_MODULE_DIR "'";
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
 * \return Number of modules loaded by this call
 */
unsigned int IPAManager::addDir(const char *libDir)
{
	struct dirent *ent;
	DIR *dir;

	dir = opendir(libDir);
	if (!dir)
		return 0;

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

		LOG(IPAManager, Debug) << "Loaded IPA module '" << path << "'";

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

	struct ipa_context *ctx = m->createContext();
	if (!ctx)
		return nullptr;

	return std::make_unique<IPAContextWrapper>(ctx);
}

} /* namespace libcamera */
