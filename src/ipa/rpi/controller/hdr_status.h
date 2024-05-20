/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023 Raspberry Pi Ltd
 *
 * HDR control algorithm status
 */
#pragma once

#include <string>

/*
 * The HDR algorithm process method should post an HdrStatus into the image
 * metadata under the tag "hdr.status".
 */

struct HdrStatus {
	std::string mode;
	std::string channel;
};
