/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024-2025 Red Hat, inc.
 *
 * Global configuration handling
 */

#include "libcamera/internal/global_configuration.h"

#include <array>
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

class EnvironmentProcessor
{
public:
	virtual ~EnvironmentProcessor() = default;

	virtual void process(ValueNode &node, const char *env) = 0;
};

/* A processor that sets a fixed value. */
template<typename T>
class EnvironmentFixedProcessor : public EnvironmentProcessor
{
public:
	EnvironmentFixedProcessor(const T &value)
		: value_(value)
	{
	}

	void process(ValueNode &node, [[maybe_unused]] const char *env) override
	{
		node.set(value_);
	}

private:
	T value_;
};

/*
 * A processor that parses the environment variable as a list of strings with a
 * custom delimiter.
 */
class EnvironmentListProcessor : public EnvironmentProcessor
{
public:
	EnvironmentListProcessor(const char *delimiter)
		: delimiter_(delimiter)
	{
	}

	void process(ValueNode &node, const char *env) override
	{
		for (auto &&value : utils::split(env, delimiter_))
			node.add(std::make_unique<ValueNode>(std::move(value)));
	}

private:
	const std::string delimiter_;
};

/* A processor that copies the value of the environment variable. */
class EnvironmentValueProcessor : public EnvironmentProcessor
{
public:
	void process(ValueNode &node, const char *env) override
	{
		node.set(std::string{ env });
	}
};

struct EnvironmentOverride {
	const char *variable;
	std::initializer_list<std::string_view> path;
	std::unique_ptr<EnvironmentProcessor> processor;
};

const std::array<EnvironmentOverride, 6> environmentOverrides{ {
	{
		"LIBCAMERA_IPA_CONFIG_PATH",
		{ "ipa", "config_paths" },
		std::make_unique<EnvironmentListProcessor>(":"),
	}, {
		"LIBCAMERA_IPA_FORCE_ISOLATION",
		{ "ipa", "force_isolation" },
		std::make_unique<EnvironmentFixedProcessor<bool>>(true),
	}, {
		"LIBCAMERA_IPA_MODULE_PATH",
		{ "ipa", "module_paths" },
		std::make_unique<EnvironmentListProcessor>(":"),
	}, {
		"LIBCAMERA_IPA_PROXY_PATH",
		{ "ipa", "proxy_paths" },
		std::make_unique<EnvironmentListProcessor>(":"),
	}, {
		"LIBCAMERA_PIPELINES_MATCH_LIST",
		{ "pipelines_match_list" },
		std::make_unique<EnvironmentListProcessor>(","),
	}, {
		"LIBCAMERA_SOFTISP_MODE",
		{ "software_isp", "mode" },
		std::make_unique<EnvironmentValueProcessor>(),
	},
} };

} /* namespace */

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
 * The configuration can be accessed using the provided helpers, namely option()
 * and listOption() to access individual options, or configuration() to access
 * the whole configuration.
 */

/**
 * \brief Initialize the global configuration
 */
GlobalConfiguration::GlobalConfiguration()
{
	load();

	if (configuration_->isEmpty()) {
		configuration_->add("version", std::make_unique<ValueNode>(1));
		configuration_->add("configuration", std::make_unique<ValueNode>());
	}

	/* Process environment variables that override configuration options. */
	ValueNode *cfg = configuration_->at("configuration");

	for (const EnvironmentOverride &envOverride : environmentOverrides) {
		const char *envValue = utils::secure_getenv(envOverride.variable);
		if (!envValue || !envValue[0])
			continue;

		std::unique_ptr<ValueNode> node = std::make_unique<ValueNode>();
		envOverride.processor->process(*node.get(), envValue);

		cfg->erase(envOverride.path);

		if (!cfg->add(envOverride.path, std::move(node)))
			LOG(Configuration, Error)
				<< "Failed to override "
				<< utils::join(envOverride.path, "/");
	}
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

	std::unique_ptr<ValueNode> configuration = YamlParser::parse(file);
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

	configuration_ = std::move(configuration);
	return true;
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
	return (*configuration_)["version"].get<unsigned int>().value_or(0);
}

/**
 * \brief Retrieve the libcamera global configuration
 *
 * This returns the whole configuration stored in the top-level section
 * `%configuration` of the YAML configuration file.
 *
 * The requested part of the configuration can be accessed using \a ValueNode
 * methods.
 *
 * \return The global configuration
 */
const ValueNode &GlobalConfiguration::configuration() const
{
	return (*configuration_)["configuration"];
}

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
	const ValueNode *c = &configuration();
	for (auto part : confPath) {
		c = &(*c)[part];
		if (!*c)
			return {};
	}
	return c->get<std::vector<std::string>>();
}

} /* namespace libcamera */
