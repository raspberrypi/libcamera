/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_manager.h - Image Processing Algorithm module manager
 */
#ifndef __LIBCAMERA_INTERNAL_IPA_MANAGER_H__
#define __LIBCAMERA_INTERNAL_IPA_MANAGER_H__

#include <stdint.h>
#include <vector>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/pub_key.h"

namespace libcamera {

class IPAManager
{
public:
	IPAManager();
	~IPAManager();

	static std::unique_ptr<IPAProxy> createIPA(PipelineHandler *pipe,
						   uint32_t maxVersion,
						   uint32_t minVersion);

private:
	static IPAManager *self_;

	void parseDir(const char *libDir, unsigned int maxDepth,
		      std::vector<std::string> &files);
	unsigned int addDir(const char *libDir, unsigned int maxDepth = 0);

	bool isSignatureValid(IPAModule *ipa) const;

	std::vector<IPAModule *> modules_;

#if HAVE_IPA_PUBKEY
	static const uint8_t publicKeyData_[];
	static const PubKey pubKey_;
#endif
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_IPA_MANAGER_H__ */
