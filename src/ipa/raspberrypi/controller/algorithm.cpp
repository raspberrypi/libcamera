/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * algorithm.cpp - ISP control algorithms
 */

#include "algorithm.hpp"

using namespace RPiController;

void Algorithm::Read([[maybe_unused]] boost::property_tree::ptree const &params)
{
}

void Algorithm::Initialise() {}

void Algorithm::SwitchMode([[maybe_unused]] CameraMode const &camera_mode,
			   [[maybe_unused]] Metadata *metadata)
{
}

void Algorithm::Prepare([[maybe_unused]] Metadata *image_metadata)
{
}

void Algorithm::Process([[maybe_unused]] StatisticsPtr &stats,
			[[maybe_unused]] Metadata *image_metadata)
{
}

// For registering algorithms with the system:

static std::map<std::string, AlgoCreateFunc> algorithms;
std::map<std::string, AlgoCreateFunc> const &RPiController::GetAlgorithms()
{
	return algorithms;
}

RegisterAlgorithm::RegisterAlgorithm(char const *name,
				     AlgoCreateFunc create_func)
{
	algorithms[std::string(name)] = create_func;
}
