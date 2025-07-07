/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022, Raspberry Pi Ltd
 *
 * af_algorithm.hpp - auto focus algorithm interface
 */
#pragma once

#include <optional>

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
	 * Where a ControlList combines a change of AfMode with other AF
	 * controls, setMode() should be called first, to ensure the
	 * algorithm will be in the correct state to handle controls.
	 *
	 * setLensPosition() returns true if the mode was AfModeManual and
	 * the lens position has changed, otherwise returns false. When it
	 * returns true, hwpos should be sent immediately to the lens driver.
	 *
	 * getMode() is provided mainly for validating controls.
	 * getLensPosition() is provided for populating DeviceStatus.
	 *
	 * getDefaultlensPosition() and getLensLimits() were added for
	 * populating ControlInfoMap. They return the static API limits
	 * which should be independent of the current range or mode.
	 */

	enum AfRange { AfRangeNormal = 0,
		       AfRangeMacro,
		       AfRangeFull,
		       AfRangeMax };

	enum AfSpeed { AfSpeedNormal = 0,
		       AfSpeedFast,
		       AfSpeedMax };

	enum AfMode { AfModeManual = 0,
		      AfModeAuto,
		      AfModeContinuous };

	enum AfPause { AfPauseImmediate = 0,
		       AfPauseDeferred,
		       AfPauseResume };

	virtual void setRange([[maybe_unused]] AfRange range)
	{
	}
	virtual void setSpeed([[maybe_unused]] AfSpeed speed)
	{
	}
	virtual void setMetering([[maybe_unused]] bool use_windows)
	{
	}
	virtual void setWindows([[maybe_unused]] libcamera::Span<libcamera::Rectangle const> const &wins)
	{
	}
	virtual void setMode(AfMode mode) = 0;
	virtual AfMode getMode() const = 0;
	virtual double getDefaultLensPosition() const = 0;
	virtual void getLensLimits(double &min, double &max) const = 0;
	virtual bool setLensPosition(double dioptres, int32_t *hwpos, bool force = false) = 0;
	virtual std::optional<double> getLensPosition() const = 0;
	virtual void triggerScan() = 0;
	virtual void cancelScan() = 0;
	virtual void pause(AfPause pause) = 0;
};

} // namespace RPiController
