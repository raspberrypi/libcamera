/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_dummy.cpp - Dummy Image Processing Algorithm module
 */

#include <iostream>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

namespace libcamera {

class IPADummy : public IPAInterface
{
public:
	int init();
};

int IPADummy::init()
{
	std::cout << "initializing dummy IPA!" << std::endl;
	return 0;
}

/*
 * External IPA module interface
 */

extern "C" {
const struct IPAModuleInfo ipaModuleInfo = {
	IPA_MODULE_API_VERSION,
	0,
	"PipelineHandlerVimc",
	"Dummy IPA for Vimc",
};

IPAInterface *ipaCreate()
{
	return new IPADummy();
}
};

}; /* namespace libcamera */
