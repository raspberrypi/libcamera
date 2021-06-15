/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * controller.cpp - ISP controller
 */

#include <libcamera/base/log.h>

#include "algorithm.hpp"
#include "controller.hpp"

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiController)

Controller::Controller()
	: switch_mode_called_(false) {}

Controller::Controller(char const *json_filename)
	: switch_mode_called_(false)
{
	Read(json_filename);
	Initialise();
}

Controller::~Controller() {}

void Controller::Read(char const *filename)
{
	boost::property_tree::ptree root;
	boost::property_tree::read_json(filename, root);
	for (auto const &key_and_value : root) {
		Algorithm *algo = CreateAlgorithm(key_and_value.first.c_str());
		if (algo) {
			algo->Read(key_and_value.second);
			algorithms_.push_back(AlgorithmPtr(algo));
		} else
			LOG(RPiController, Warning)
				<< "No algorithm found for \"" << key_and_value.first << "\"";
	}
}

Algorithm *Controller::CreateAlgorithm(char const *name)
{
	auto it = GetAlgorithms().find(std::string(name));
	return it != GetAlgorithms().end() ? (*it->second)(this) : nullptr;
}

void Controller::Initialise()
{
	for (auto &algo : algorithms_)
		algo->Initialise();
}

void Controller::SwitchMode(CameraMode const &camera_mode, Metadata *metadata)
{
	for (auto &algo : algorithms_)
		algo->SwitchMode(camera_mode, metadata);
	switch_mode_called_ = true;
}

void Controller::Prepare(Metadata *image_metadata)
{
	assert(switch_mode_called_);
	for (auto &algo : algorithms_)
		if (!algo->IsPaused())
			algo->Prepare(image_metadata);
}

void Controller::Process(StatisticsPtr stats, Metadata *image_metadata)
{
	assert(switch_mode_called_);
	for (auto &algo : algorithms_)
		if (!algo->IsPaused())
			algo->Process(stats, image_metadata);
}

Metadata &Controller::GetGlobalMetadata()
{
	return global_metadata_;
}

Algorithm *Controller::GetAlgorithm(std::string const &name) const
{
	// The passed name must be the entire algorithm name, or must match the
	// last part of it with a period (.) just before.
	size_t name_len = name.length();
	for (auto &algo : algorithms_) {
		char const *algo_name = algo->Name();
		size_t algo_name_len = strlen(algo_name);
		if (algo_name_len >= name_len &&
		    strcasecmp(name.c_str(),
			       algo_name + algo_name_len - name_len) == 0 &&
		    (name_len == algo_name_len ||
		     algo_name[algo_name_len - name_len - 1] == '.'))
			return algo.get();
	}
	return nullptr;
}
