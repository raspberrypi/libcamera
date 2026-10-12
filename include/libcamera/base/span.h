/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * C++20 std::span<> implementation for C++11
 */

#pragma once

#include <span>

/* \todo Remove this header. */

namespace libcamera {

template<typename T, std::size_t Extent = std::dynamic_extent>
using Span [[deprecated("use std::span instead")]] = std::span<T, Extent>;

} /* namespace libcamera */
