/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Red Hat, Inc.
 *
 * Software ISP control algorithm interface
 */

#pragma once

#include <libipa/algorithm.h>

#include "module.h"

namespace libcamera {

namespace ipa::softisp {

using Algorithm = libcamera::ipa::Algorithm<Module>;

} /* namespace ipa::softisp */

} /* namespace libcamera */
