/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af_algorithm.hpp - auto focus algorithm interface
 */
#pragma once

#include <libcamera/base/span.h>

#include "algorithm.h"

namespace RPiController {

class AfAlgorithm : public Algorithm
{
public:
	AfAlgorithm(Controller *controller)
		: Algorithm(controller) {}

	/*
	 * An autofocus algorithm should provide the following calls.
	 *
	 * When setLensPosition() returns true, the hardware lens position
	 * should be forwarded immediately to the lens driver.
	 *
	 * This API is simpler and more permissive than libcamera's model:
	 * to emulate the libcamera interface, some controls must be ignored
	 * in certain modes. AfPause may be implemented by disabling CAF.
	 */

	enum AfRange { AF_RANGE_NORMAL = 0,
		       AF_RANGE_MACRO,
		       AF_RANGE_FULL,
		       AF_NUM_RANGES };

	enum AfSpeed { AF_SPEED_NORMAL = 0,
		       AF_SPEED_FAST,
		       AF_NUM_SPEEDS };

	virtual void setRange(AfRange range) { (void)range; }
	virtual void setSpeed(AfSpeed speed) { (void)speed; }
	virtual void setMetering(bool use_windows) { (void)use_windows; }
	virtual void setWindows(libcamera::Span<libcamera::Rectangle const> const &wins) { (void)wins; }

	virtual bool setLensPosition(double dioptres, int32_t *hwpos) = 0;
	virtual void enableCAF(bool enable) = 0;
	virtual void triggerScan() = 0;
	virtual void cancelScan() = 0;
};

} // namespace RPiController
