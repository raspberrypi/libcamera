/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * controller.cpp - ISP controller
 */

#include <assert.h>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include "libcamera/internal/yaml_parser.h"

#include "algorithm.h"
#include "controller.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiController)

Controller::Controller()
	: switchModeCalled_(false)
{
}

Controller::Controller(char const *jsonFilename)
	: switchModeCalled_(false)
{
	read(jsonFilename);
	initialise();
}

Controller::~Controller() {}

int Controller::read(char const *filename)
{
	File file(filename);
	if (!file.open(File::OpenModeFlag::ReadOnly)) {
		LOG(RPiController, Warning)
			<< "Failed to open tuning file '" << filename << "'";
		return -EINVAL;
	}

	std::unique_ptr<YamlObject> root = YamlParser::parse(file);

	for (auto const &[key, value] : root->asDict()) {
		Algorithm *algo = createAlgorithm(key.c_str());
		if (algo) {
			int ret = algo->read(value);
			if (ret)
				return ret;
			algorithms_.push_back(AlgorithmPtr(algo));
		} else
			LOG(RPiController, Warning)
				<< "No algorithm found for \"" << key << "\"";
	}

	return 0;
}

Algorithm *Controller::createAlgorithm(char const *name)
{
	auto it = getAlgorithms().find(std::string(name));
	return it != getAlgorithms().end() ? (*it->second)(this) : nullptr;
}

void Controller::initialise()
{
	for (auto &algo : algorithms_)
		algo->initialise();
}

void Controller::switchMode(CameraMode const &cameraMode, Metadata *metadata)
{
	for (auto &algo : algorithms_)
		algo->switchMode(cameraMode, metadata);
	switchModeCalled_ = true;
}

void Controller::prepare(Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		if (!algo->isPaused())
			algo->prepare(imageMetadata);
}

void Controller::process(StatisticsPtr stats, Metadata *imageMetadata)
{
	assert(switchModeCalled_);
	for (auto &algo : algorithms_)
		if (!algo->isPaused())
			algo->process(stats, imageMetadata);
}

Metadata &Controller::getGlobalMetadata()
{
	return globalMetadata_;
}

Algorithm *Controller::getAlgorithm(std::string const &name) const
{
	/*
	 * The passed name must be the entire algorithm name, or must match the
	 * last part of it with a period (.) just before.
	 */
	size_t nameLen = name.length();
	for (auto &algo : algorithms_) {
		char const *algoName = algo->name();
		size_t algoNameLen = strlen(algoName);
		if (algoNameLen >= nameLen &&
		    strcasecmp(name.c_str(),
			       algoName + algoNameLen - nameLen) == 0 &&
		    (nameLen == algoNameLen ||
		     algoName[algoNameLen - nameLen - 1] == '.'))
			return algo.get();
	}
	return nullptr;
}
