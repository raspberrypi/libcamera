/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module_info.h - Image Processing Algorithm module information
 */
#ifndef __LIBCAMERA_IPA_MODULE_INFO_H__
#define __LIBCAMERA_IPA_MODULE_INFO_H__

#ifdef __cplusplus
namespace libcamera {
#endif

struct IPAModuleInfo {
	char name[256];
	unsigned int version;
};

#ifdef __cplusplus
extern "C" {
#endif
extern const struct IPAModuleInfo ipaModuleInfo;
#ifdef __cplusplus
};
#endif

#ifdef __cplusplus
}; /* namespace libcamera */
#endif

#endif /* __LIBCAMERA_IPA_MODULE_INFO_H__ */
