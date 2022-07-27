/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * lux_status.h - Lux control algorithm status
 */
#pragma once

/*
 * The "lux" algorithm looks at the (AGC) histogram statistics of the frame and
 * estimates the current lux level of the scene. It does this by a simple ratio
 * calculation comparing to a reference image that was taken in known conditions
 * with known statistics and a properly measured lux level. There is a slight
 * problem with aperture, in that it may be variable without the system knowing
 * or being aware of it. In this case an external application may set a
 * "current_aperture" value if it wishes, which would be used in place of the
 * (presumably meaningless) value in the image metadata.
 */

struct LuxStatus {
	double lux;
	double aperture;
};
