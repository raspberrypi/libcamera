/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_dummy_isolate.cpp - Dummy Image Processing Algorithm module that needs
 * to be isolated
 */

#include <iostream>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

namespace libcamera {

class IPADummyIsolate : public IPAInterface
{
public:
	int init();
};

int IPADummyIsolate::init()
{
	std::cout << "initializing isolated dummy IPA!" << std::endl;
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
	"Dummy IPA for Vimc that needs to be isolated",
	"Proprietary",
};

IPAInterface *ipaCreate()
{
	return new IPADummyIsolate();
}
};

}; /* namespace libcamera */
