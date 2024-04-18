/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * black_level.h - black level handling
 */

#pragma once

#include <array>

#include "libcamera/internal/software_isp/swisp_stats.h"

namespace libcamera {

class BlackLevel
{
public:
	BlackLevel();
	unsigned int get() const;
	void update(SwIspStats::Histogram &yHistogram);

private:
	unsigned int blackLevel_;
	bool blackLevelSet_;
};

} /* namespace libcamera */
