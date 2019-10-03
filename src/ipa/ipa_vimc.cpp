/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_vimc.cpp - Vimc Image Processing Algorithm module
 */

#include <iostream>

#include <ipa/ipa_interface.h>
#include <ipa/ipa_module_info.h>

namespace libcamera {

class IPAVimc : public IPAInterface
{
public:
	int init();
};

int IPAVimc::init()
{
	std::cout << "initializing vimc IPA!" << std::endl;
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
	LICENSE,
};

IPAInterface *ipaCreate()
{
	return new IPAVimc();
}
};

}; /* namespace libcamera */
