/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Authors:
 * Hans de Goede <hdegoede@redhat.com>
 *
 * Simple builtin benchmark to measure software ISP processing times
 */

#pragma once

#include <stdint.h>
#include <time.h>
#include <libcamera/base/log.h>
#include "libcamera/internal/global_configuration.h"

namespace libcamera {

class Benchmark
{
public:
	Benchmark(const GlobalConfiguration &configuration);
	~Benchmark();

	void startFrame(void);
	void finishFrame(void);

private:
	timespec frameStartTime_;
	bool measure;
	/* Skip 30 frames for things to stabilize then measure 30 frames */
	unsigned int encounteredFrames_ = 0;
	int64_t frameProcessTime_ = 0;
	unsigned int skipBeforeMeasure_ = 30;
	unsigned int framesToMeasure_ = 30;
};

} /* namespace libcamera */
