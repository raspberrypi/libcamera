/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * debayer_params.h - DebayerParams header
 */

#pragma once

namespace libcamera {

struct DebayerParams {
	static constexpr unsigned int kGain10 = 256;

	unsigned int gainR;
	unsigned int gainG;
	unsigned int gainB;

	float gamma;
	/**
	 * \brief Level of the black point, 0..255, 0 is no correction.
	 */
	unsigned int blackLevel;
};

} /* namespace libcamera */
