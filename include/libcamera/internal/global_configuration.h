/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat, inc.
 *
 * Global configuration handling
 */

#pragma once

#include <filesystem>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>

#include <libcamera/base/utils.h>

#include "libcamera/internal/value_node.h"

namespace libcamera {

class GlobalConfiguration
{
public:
	GlobalConfiguration();

	unsigned int version() const;
	const ValueNode &configuration() const;

	template<typename T>
	std::optional<T> option(
		const std::initializer_list<std::string_view> confPath) const
	{
		const ValueNode *c = &configuration();
		for (auto part : confPath) {
			c = &(*c)[part];
			if (!*c)
				return {};
		}
		return c->get<T>();
	}

	std::optional<std::vector<std::string>> listOption(
		const std::initializer_list<std::string_view> confPath) const;

private:
	void load();
	bool loadFile(const std::filesystem::path &fileName);

	std::unique_ptr<ValueNode> configuration_ =
		std::make_unique<ValueNode>();
};

} /* namespace libcamera */
