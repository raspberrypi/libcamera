/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * algorithm.cpp - ISP control algorithms
 */

#include "algorithm.hpp"

using namespace RPi;

void Algorithm::Read(boost::property_tree::ptree const &params)
{
	(void)params;
}

void Algorithm::Initialise() {}

void Algorithm::SwitchMode(CameraMode const &camera_mode)
{
	(void)camera_mode;
}

void Algorithm::Prepare(Metadata *image_metadata)
{
	(void)image_metadata;
}

void Algorithm::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	(void)stats;
	(void)image_metadata;
}

// For registering algorithms with the system:

static std::map<std::string, AlgoCreateFunc> algorithms;
std::map<std::string, AlgoCreateFunc> const &RPi::GetAlgorithms()
{
	return algorithms;
}

RegisterAlgorithm::RegisterAlgorithm(char const *name,
				     AlgoCreateFunc create_func)
{
	algorithms[std::string(name)] = create_func;
}
