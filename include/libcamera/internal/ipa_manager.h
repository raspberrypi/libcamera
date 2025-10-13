/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * Image Processing Algorithm module manager
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/pub_key.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAManager)

class CameraManager;
class GlobalConfiguration;
class IPAModule;
class PipelineHandler;

class IPAManager
{
public:
	IPAManager(const CameraManager &cm);
	~IPAManager();

	template<typename T>
	std::unique_ptr<T> createIPA(PipelineHandler *pipe, uint32_t minVersion,
				     uint32_t maxVersion)
	{
		IPAModule *m = module(pipe, minVersion, maxVersion);
		if (!m)
			return nullptr;

		auto proxy = [&]() -> std::unique_ptr<T> {
			if (isSignatureValid(m))
				return std::make_unique<typename T::Threaded>(m, cm_);
			else
				return std::make_unique<typename T::Isolated>(m, cm_);
		}();

		if (!proxy->isValid()) {
			LOG(IPAManager, Error) << "Failed to load proxy";
			return nullptr;
		}

		return proxy;
	}

#if HAVE_IPA_PUBKEY
	static const PubKey &pubKey()
	{
		return pubKey_;
	}
#endif

private:
	void parseDir(const char *libDir, unsigned int maxDepth,
		      std::vector<std::string> &files);
	unsigned int addDir(const char *libDir, unsigned int maxDepth = 0);

	IPAModule *module(PipelineHandler *pipe, uint32_t minVersion,
			  uint32_t maxVersion);

	bool isSignatureValid(IPAModule *ipa) const;

	const CameraManager &cm_;
	std::vector<std::unique_ptr<IPAModule>> modules_;

#if HAVE_IPA_PUBKEY
	static const uint8_t publicKeyData_[];
	static const PubKey pubKey_;
	bool forceIsolation_;
#endif
};

} /* namespace libcamera */
