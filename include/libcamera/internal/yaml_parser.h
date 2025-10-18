/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * libcamera YAML parsing helper
 */

#pragma once

#include <memory>

#include "libcamera/internal/yaml_object.h"

namespace libcamera {

class File;

class YamlParser final
{
public:
	static std::unique_ptr<YamlObject> parse(File &file);
};

} /* namespace libcamera */
