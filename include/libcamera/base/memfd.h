/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas on Board Oy
 *
 * Anonymous file creation
 */

#pragma once

#include <libcamera/base/flags.h>
#include <libcamera/base/unique_fd.h>

namespace libcamera {

class MemFd
{
public:
	enum class Seal {
		None = 0,
		Shrink = (1 << 0),
		Grow = (1 << 1),
	};

	using Seals = Flags<Seal>;

	static UniqueFD create(const char *name, std::size_t size,
			       Seals seals = Seal::None);
};

LIBCAMERA_FLAGS_ENABLE_OPERATORS(MemFd::Seal)

} /* namespace libcamera */
