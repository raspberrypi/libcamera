/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat, inc.
 *
 * Global configuration handling
 */

#include "libcamera/internal/global_configuration.h"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <sys/types.h>
#include <vector>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

namespace libcamera {

namespace {
const std::vector<std::filesystem::path> globalConfigurationFiles = {
	std::filesystem::path(LIBCAMERA_SYSCONF_DIR) / "configuration.yaml",
	std::filesystem::path(LIBCAMERA_DATA_DIR) / "configuration.yaml",
};
}

LOG_DEFINE_CATEGORY(Configuration)

/**
 * \class GlobalConfiguration
 * \brief Support for global libcamera configuration
 *
 * The configuration file is a YAML file and the configuration itself is stored
 * under a `%configuration` top-level item.
 *
 * The configuration file is looked up in the user's home directory first and,
 * if it is not found, then in system-wide configuration directories. If
 * multiple configuration files exist then only the first one found is used and
 * no configuration merging is performed.
 *
 * If the first found configuration file cannot be opened or parsed, an error is
 * reported and no configuration file is used. This is to prevent libcamera from
 * using an unintended configuration file.
 *
 * The configuration can be accessed using the provided helpers, namely
 * option(), envOption(), listOption() and envListOption() to access individual
 * options, or configuration() to access the whole configuration.
 */

bool GlobalConfiguration::loadFile(const std::filesystem::path &fileName)
{
	File file(fileName);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		if (file.error() == -ENOENT)
			return false;

		LOG(Configuration, Error)
			<< "Failed to open configuration file " << fileName;
		return true;
	}

	std::unique_ptr<YamlObject> configuration = YamlParser::parse(file);
	if (!configuration) {
		LOG(Configuration, Error)
			<< "Failed to parse configuration file " << fileName;
		return true;
	}

	const std::optional<int> version = (*configuration)["version"].get<int>();
	if (version != 1) {
		LOG(Configuration, Error)
			<< "Failed to load configuration file due to unsupported version "
			<< (version ? std::to_string(version.value()) : "\"unspecified\"")
			<< ", expected version 1";
		return true;
	}

	yamlConfiguration_ = std::move(configuration);
	return true;
}

void GlobalConfiguration::load()
{
	std::filesystem::path userConfigurationDirectory;
	const char *xdgConfigHome = utils::secure_getenv("XDG_CONFIG_HOME");
	if (xdgConfigHome) {
		userConfigurationDirectory = xdgConfigHome;
	} else {
		const char *home = utils::secure_getenv("HOME");
		if (home)
			userConfigurationDirectory =
				std::filesystem::path(home) / ".config";
	}

	if (!userConfigurationDirectory.empty()) {
		std::filesystem::path user_configuration_file =
			userConfigurationDirectory / "libcamera" / "configuration.yaml";
		if (loadFile(user_configuration_file))
			return;
	}

	for (const auto &path : globalConfigurationFiles) {
		if (loadFile(path))
			return;
	}
}

/**
 * \brief Initialize the global configuration
 */
GlobalConfiguration::GlobalConfiguration()
{
	load();
}

/**
 * \typedef GlobalConfiguration::Configuration
 * \brief Type representing global libcamera configuration
 *
 * All code outside GlobalConfiguration must use this type declaration and not
 * the underlying type.
 */

/**
 * \fn std::optional<T> GlobalConfiguration::option(const std::initializer_list<std::string_view> &confPath) const
 * \brief Retrieve the value of configuration option \a confPath
 * \tparam T The type of the retrieved configuration value
 * \param[in] confPath Sequence of the YAML section names (excluding
 * `%configuration`) leading to the requested option
 * \return The value of the configuration item corresponding to \a confPath if
 * it exists in the configuration file, or no value otherwise
 */

/**
 * \brief Retrieve the value of configuration option \a confPath
 * \param[in] confPath Sequence of the YAML section names (excluding
 * `%configuration`) leading to the requested list option, separated by dots
 * \return A vector of strings or no value if not found
 */
std::optional<std::vector<std::string>> GlobalConfiguration::listOption(
	const std::initializer_list<std::string_view> confPath) const
{
	const YamlObject *c = &configuration();
	for (auto part : confPath) {
		c = &(*c)[part];
		if (!*c)
			return {};
	}
	return c->getList<std::string>();
}

/**
 * \brief Retrieve the value of environment variable with a fallback on the configuration file
 * \param[in] envVariable Environment variable to get the value from
 * \param[in] confPath The sequence of YAML section names to fall back on when
 * \a envVariable is unavailable
 *
 * This helper looks first at the given environment variable and if it is
 * defined then it returns its value (even if it is empty). Otherwise it looks
 * for \a confPath the same way as in GlobalConfiguration::option. Only string
 * values are supported.
 *
 * \note Support for using environment variables to configure libcamera behavior
 * is provided here mostly for backward compatibility reasons. Introducing new
 * configuration environment variables is discouraged.
 *
 * \return The value retrieved from the given environment if it is set,
 * otherwise the value from the configuration file if it exists, or no value if
 * it does not
 */
std::optional<std::string> GlobalConfiguration::envOption(
	const char *envVariable,
	const std::initializer_list<std::string_view> confPath) const
{
	const char *envValue = utils::secure_getenv(envVariable);
	if (envValue)
		return std::optional{ std::string{ envValue } };
	return option<std::string>(confPath);
}

/**
 * \brief Retrieve the value of the configuration option from a file or environment
 * \param[in] envVariable Environment variable to get the value from
 * \param[in] confPath The same as in GlobalConfiguration::option
 * \param[in] delimiter Items separator in the environment variable
 *
 * This helper looks first at the given environment variable and if it is
 * defined (even if it is empty) then it splits its value by semicolons and
 * returns the resulting list of strings. Otherwise it looks for \a confPath the
 * same way as in GlobalConfiguration::option, value of which must be a list of
 * strings.
 *
 * \note Support for using environment variables to configure libcamera behavior
 * is provided here mostly for backward compatibility reasons. Introducing new
 * configuration environment variables is discouraged.
 *
 * \return A vector of strings retrieved from the given environment option or
 * configuration file or no value if not found; the vector may be empty
 */
std::optional<std::vector<std::string>> GlobalConfiguration::envListOption(
	const char *const envVariable,
	const std::initializer_list<std::string_view> confPath,
	const std::string delimiter) const
{
	const char *envValue = utils::secure_getenv(envVariable);
	if (envValue) {
		auto items = utils::split(envValue, delimiter);
		return std::vector<std::string>(items.begin(), items.end());
	}
	return listOption(confPath);
}

/**
 * \brief Retrieve the configuration version
 *
 * The version is declared in the configuration file in the top-level `%version`
 * element, alongside `%configuration`. This has currently no real use but may be
 * needed in future if configuration incompatibilities occur.
 *
 * \return Configuration version as declared in the configuration file or 0 if
 * no global configuration is available
 */
unsigned int GlobalConfiguration::version() const
{
	return (*yamlConfiguration_)["version"].get<unsigned int>().value_or(0);
}

/**
 * \brief Retrieve the libcamera global configuration
 *
 * This returns the whole configuration stored in the top-level section
 * `%configuration` of the YAML configuration file.
 *
 * The requested part of the configuration can be accessed using \a YamlObject
 * methods.
 *
 * \note \a YamlObject type itself shouldn't be used in type declarations to
 * avoid trouble if we decide to change the underlying data objects in future.
 *
 * \return The whole configuration section
 */
GlobalConfiguration::Configuration GlobalConfiguration::configuration() const
{
	return (*yamlConfiguration_)["configuration"];
}

} /* namespace libcamera */
