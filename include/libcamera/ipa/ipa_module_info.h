/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module_info.h - Image Processing Algorithm module information
 */
#ifndef __LIBCAMERA_IPA_MODULE_INFO_H__
#define __LIBCAMERA_IPA_MODULE_INFO_H__

#include <stdint.h>

#define IPA_MODULE_API_VERSION 1

namespace libcamera {

struct IPAModuleInfo {
	int moduleAPIVersion;
	uint32_t pipelineVersion;
	char pipelineName[256];
	char name[256];
} __attribute__((packed));

extern "C" {
extern const struct IPAModuleInfo ipaModuleInfo;
}

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_MODULE_INFO_H__ */
