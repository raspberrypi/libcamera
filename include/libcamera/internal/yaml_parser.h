/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * libcamera YAML parsing helper
 */

#pragma once

#include <memory>

#include "libcamera/internal/value_node.h"

namespace libcamera {

class File;

class YamlParser final
{
public:
	static std::unique_ptr<ValueNode> parse(File &file);
};

} /* namespace libcamera */
