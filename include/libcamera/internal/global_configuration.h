/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat, inc.
 *
 * Global configuration handling
 */

#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

class GlobalConfiguration
{
public:
	using Configuration = const YamlObject &;

	GlobalConfiguration();

	unsigned int version() const;
	Configuration configuration() const;

	template<typename T>
	std::optional<T> option(
		const std::initializer_list<std::string_view> confPath) const
	{
		const YamlObject *c = &configuration();
		for (auto part : confPath) {
			c = &(*c)[part];
			if (!*c)
				return {};
		}
		return c->get<T>();
	}

	std::optional<std::vector<std::string>> listOption(
		const std::initializer_list<std::string_view> confPath) const;
	std::optional<std::string> envOption(
		const char *const envVariable,
		const std::initializer_list<std::string_view> confPath) const;
	std::optional<std::vector<std::string>> envListOption(
		const char *const envVariable,
		const std::initializer_list<std::string_view> confPath,
		const std::string delimiter = ":") const;

private:
	bool loadFile(const std::filesystem::path &fileName);
	void load();

	std::unique_ptr<YamlObject> yamlConfiguration_ =
		std::make_unique<YamlObject>();
};

} /* namespace libcamera */
