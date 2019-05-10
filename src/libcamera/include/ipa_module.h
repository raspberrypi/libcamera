/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_module.h - Image Processing Algorithm module
 */
#ifndef __LIBCAMERA_IPA_MODULE_H__
#define __LIBCAMERA_IPA_MODULE_H__

#include <string>

#include <libcamera/ipa/ipa_module_info.h>

namespace libcamera {

class IPAModule
{
public:
	explicit IPAModule(const std::string &libPath);

	bool isValid() const;

	const struct IPAModuleInfo &info() const;

private:
	struct IPAModuleInfo info_;

	std::string libPath_;
	bool valid_;

	int loadIPAModuleInfo();
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_MODULE_H__ */
