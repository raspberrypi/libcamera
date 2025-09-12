/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat, inc.
 *
 * Global configuration handling
 */

#include "libcamera/internal/global_configuration.h"

#include <filesystem>
#include <string_view>
#include <sys/types.h>

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
 * under a `configuration' top-level item.
 *
 * The configuration file is looked up in the user's home directory first and,
 * if it is not found, then in system-wide configuration directories. If
 * multiple configuration files exist then only the first one found is used and
 * no configuration merging is performed.
 *
 * If the first found configuration file cannot be opened or parsed, an error is
 * reported and no configuration file is used. This is to prevent libcamera from
 * using an unintended configuration file.
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
 * \brief Retrieve the configuration version
 *
 * The version is declared in the configuration file in the top-level `version'
 * element, alongside `configuration'. This has currently no real use but may be
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
 * `configuration' of the YAML configuration file.
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
