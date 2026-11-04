/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Ideas On Board Oy
 *
 * Image orientation
 */

#pragma once

#include <iostream>
#include <optional>

namespace libcamera {

enum class Orientation {
	/* EXIF tag 274 starts from '1' */
	Rotate0 = 1,
	Rotate0Mirror,
	Rotate180,
	Rotate180Mirror,
	Rotate90Mirror,
	Rotate270,
	Rotate270Mirror,
	Rotate90,
};

std::optional<Orientation> orientationFromRotation(int angle);

std::ostream &operator<<(std::ostream &out, const Orientation &orientation);

} /* namespace libcamera */
