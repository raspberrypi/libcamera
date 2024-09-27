/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * black level handling
 */

#pragma once

#include <array>
#include <stdint.h>

#include "libcamera/internal/software_isp/swisp_stats.h"

namespace libcamera {

namespace ipa::soft {

class BlackLevel
{
public:
	BlackLevel();
	uint8_t get() const;
	void update(SwIspStats::Histogram &yHistogram);

private:
	uint8_t blackLevel_;
	bool blackLevelSet_;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
