/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_manager.h - Image Processing Algorithm module manager
 */

#pragma once

#include <stdint.h>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/pub_key.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAManager)

class IPAManager
{
public:
	IPAManager();
	~IPAManager();

	template<typename T>
	static std::unique_ptr<T> createIPA(PipelineHandler *pipe,
					    uint32_t minVersion,
					    uint32_t maxVersion)
	{
		IPAModule *m = self_->module(pipe, minVersion, maxVersion);
		if (!m)
			return nullptr;

		std::unique_ptr<T> proxy = std::make_unique<T>(m, !self_->isSignatureValid(m));
		if (!proxy->isValid()) {
			LOG(IPAManager, Error) << "Failed to load proxy";
			return nullptr;
		}

		return proxy;
	}

private:
	static IPAManager *self_;

	void parseDir(const char *libDir, unsigned int maxDepth,
		      std::vector<std::string> &files);
	unsigned int addDir(const char *libDir, unsigned int maxDepth = 0);

	IPAModule *module(PipelineHandler *pipe, uint32_t minVersion,
			  uint32_t maxVersion);

	bool isSignatureValid(IPAModule *ipa) const;

	std::vector<IPAModule *> modules_;

#if HAVE_IPA_PUBKEY
	static const uint8_t publicKeyData_[];
	static const PubKey pubKey_;
#endif
};

} /* namespace libcamera */
