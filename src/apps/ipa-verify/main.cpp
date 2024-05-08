/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2023, Ideas on Board Oy
 *
 * Verify signature on an IPA module
 */

#include <iostream>
#include <libgen.h>

#include <libcamera/base/file.h>
#include <libcamera/base/span.h>

#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/ipa_module.h"

using namespace libcamera;

namespace {

bool isSignatureValid(IPAModule *ipa)
{
	File file{ ipa->path() };
	if (!file.open(File::OpenModeFlag::ReadOnly))
		return false;

	Span<uint8_t> data = file.map();
	if (data.empty())
		return false;

	return IPAManager::pubKey().verify(data, ipa->signature());
}

void usage(char *argv0)
{
	std::cout << "Usage: " << basename(argv0) << " ipa_name.so" << std::endl;
	std::cout << std::endl;
	std::cout << "Verify the signature of an IPA module. The signature file ipa_name.so.sign is" << std::endl;
	std::cout << "expected to be in the same directory as the IPA module." << std::endl;
}

} /* namespace */

int main(int argc, char **argv)
{
	if (argc != 2) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	IPAModule module{ argv[1] };
	if (!module.isValid()) {
		std::cout << "Invalid IPA module " << argv[1] << std::endl;
		return EXIT_FAILURE;
	}

	if (!isSignatureValid(&module)) {
		std::cout << "IPA module signature is invalid" << std::endl;
		return EXIT_FAILURE;
	}

	std::cout << "IPA module signature is valid" << std::endl;
	return 0;
}
