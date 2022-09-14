/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_manager.cpp - Image Processing Algorithm module manager
 */

#include "libcamera/internal/ipa_manager.h"

#include <algorithm>
#include <dirent.h>
#include <string.h>
#include <sys/types.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/pipeline_handler.h"

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
 * To create an IPA context, pipeline handlers call the IPAManager::createIPA()
 * function. For a directly loaded module, the manager calls the module's
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
 * In all cases the data passed to the IPAInterface member functions is
 * serialized to Plain Old Data, either for the purpose of passing it to the IPA
 * context plain C API, or to transmit the data to the isolated process through
 * IPC.
 */

IPAManager *IPAManager::self_ = nullptr;

/**
 * \brief Construct an IPAManager instance
 *
 * The IPAManager class is meant to only be instantiated once, by the
 * CameraManager.
 */
IPAManager::IPAManager()
{
	if (self_)
		LOG(IPAManager, Fatal)
			<< "Multiple IPAManager objects are not allowed";

#if HAVE_IPA_PUBKEY
	if (!pubKey_.isValid())
		LOG(IPAManager, Warning) << "Public key not valid";
#endif

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

	/*
	 * When libcamera is used before it is installed, load IPAs from the
	 * same build directory as the libcamera library itself.
	 */
	std::string root = utils::libcameraBuildPath();
	if (!root.empty()) {
		std::string ipaBuildPath = root + "src/ipa";
		constexpr int maxDepth = 1;

		LOG(IPAManager, Info)
			<< "libcamera is not installed. Adding '"
			<< ipaBuildPath << "' to the IPA search path";

		ipaCount += addDir(ipaBuildPath.c_str(), maxDepth);
	}

	/* Finally try to load IPAs from the installed system path. */
	ipaCount += addDir(IPA_MODULE_DIR);

	if (!ipaCount)
		LOG(IPAManager, Warning)
			<< "No IPA found in '" IPA_MODULE_DIR "'";

	self_ = this;
}

IPAManager::~IPAManager()
{
	for (IPAModule *module : modules_)
		delete module;

	self_ = nullptr;
}

/**
 * \brief Identify shared library objects within a directory
 * \param[in] libDir The directory to search for shared objects
 * \param[in] maxDepth The maximum depth of sub-directories to parse
 * \param[out] files A vector of paths to shared object library files
 *
 * Search a directory for .so files, allowing recursion down to sub-directories
 * no further than the depth specified by \a maxDepth.
 *
 * Discovered shared objects are added to the \a files vector.
 */
void IPAManager::parseDir(const char *libDir, unsigned int maxDepth,
			  std::vector<std::string> &files)
{
	struct dirent *ent;
	DIR *dir;

	dir = opendir(libDir);
	if (!dir)
		return;

	while ((ent = readdir(dir)) != nullptr) {
		if (ent->d_type == DT_DIR && maxDepth) {
			if (strcmp(ent->d_name, ".") == 0 ||
			    strcmp(ent->d_name, "..") == 0)
				continue;

			std::string subdir = std::string(libDir) + "/" + ent->d_name;

			/* Recursion is limited to maxDepth. */
			parseDir(subdir.c_str(), maxDepth - 1, files);

			continue;
		}

		int offset = strlen(ent->d_name) - 3;
		if (offset < 0)
			continue;
		if (strcmp(&ent->d_name[offset], ".so"))
			continue;

		files.push_back(std::string(libDir) + "/" + ent->d_name);
	}

	closedir(dir);
}

/**
 * \brief Load IPA modules from a directory
 * \param[in] libDir The directory to search for IPA modules
 * \param[in] maxDepth The maximum depth of sub-directories to search
 *
 * This function tries to create an IPAModule instance for every shared object
 * found in \a libDir, and skips invalid IPA modules.
 *
 * Sub-directories are searched up to a depth of \a maxDepth. A \a maxDepth
 * value of 0 only searches the directory specified in \a libDir.
 *
 * \return Number of modules loaded by this call
 */
unsigned int IPAManager::addDir(const char *libDir, unsigned int maxDepth)
{
	std::vector<std::string> files;

	parseDir(libDir, maxDepth, files);

	/* Ensure a stable ordering of modules. */
	std::sort(files.begin(), files.end());

	unsigned int count = 0;
	for (const std::string &file : files) {
		IPAModule *ipaModule = new IPAModule(file);
		if (!ipaModule->isValid()) {
			delete ipaModule;
			continue;
		}

		LOG(IPAManager, Debug) << "Loaded IPA module '" << file << "'";

		modules_.push_back(ipaModule);
		count++;
	}

	return count;
}

/**
 * \brief Retrieve an IPA module that matches a given pipeline handler
 * \param[in] pipe The pipeline handler
 * \param[in] minVersion Minimum acceptable version of IPA module
 * \param[in] maxVersion Maximum acceptable version of IPA module
 */
IPAModule *IPAManager::module(PipelineHandler *pipe, uint32_t minVersion,
			      uint32_t maxVersion)
{
	for (IPAModule *module : modules_) {
		if (module->match(pipe, minVersion, maxVersion))
			return module;
	}

	return nullptr;
}

/**
 * \fn IPAManager::createIPA()
 * \brief Create an IPA proxy that matches a given pipeline handler
 * \param[in] pipe The pipeline handler that wants a matching IPA proxy
 * \param[in] minVersion Minimum acceptable version of IPA module
 * \param[in] maxVersion Maximum acceptable version of IPA module
 *
 * \return A newly created IPA proxy, or nullptr if no matching IPA module is
 * found or if the IPA proxy fails to initialize
 */

bool IPAManager::isSignatureValid([[maybe_unused]] IPAModule *ipa) const
{
#if HAVE_IPA_PUBKEY
	char *force = utils::secure_getenv("LIBCAMERA_IPA_FORCE_ISOLATION");
	if (force && force[0] != '\0') {
		LOG(IPAManager, Debug)
			<< "Isolation of IPA module " << ipa->path()
			<< " forced through environment variable";
		return false;
	}

	File file{ ipa->path() };
	if (!file.open(File::OpenModeFlag::ReadOnly))
		return false;

	Span<uint8_t> data = file.map();
	if (data.empty())
		return false;

	bool valid = pubKey_.verify(data, ipa->signature());

	LOG(IPAManager, Debug)
		<< "IPA module " << ipa->path() << " signature is "
		<< (valid ? "valid" : "not valid");

	return valid;
#else
	return false;
#endif
}

} /* namespace libcamera */
