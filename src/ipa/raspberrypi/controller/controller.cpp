/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * controller.cpp - ISP controller
 */

#include <libcamera/base/log.h>

#include "algorithm.h"
#include "controller.h"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

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

void Controller::read(char const *filename)
{
	boost::property_tree::ptree root;
	boost::property_tree::read_json(filename, root);
	for (auto const &keyAndValue : root) {
		Algorithm *algo = createAlgorithm(keyAndValue.first.c_str());
		if (algo) {
			algo->read(keyAndValue.second);
			algorithms_.push_back(AlgorithmPtr(algo));
		} else
			LOG(RPiController, Warning)
				<< "No algorithm found for \"" << keyAndValue.first << "\"";
	}
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
