/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * algorithm.cpp - ISP control algorithms
 */

#include "algorithm.h"

using namespace RPiController;

int Algorithm::read([[maybe_unused]] const libcamera::YamlObject &params)
{
	return 0;
}

void Algorithm::initialise()
{
}

void Algorithm::switchMode([[maybe_unused]] CameraMode const &cameraMode,
			   [[maybe_unused]] Metadata *metadata)
{
}

void Algorithm::prepare([[maybe_unused]] Metadata *imageMetadata)
{
}

void Algorithm::process([[maybe_unused]] StatisticsPtr &stats,
			[[maybe_unused]] Metadata *imageMetadata)
{
}

/* For registering algorithms with the system: */

static std::map<std::string, AlgoCreateFunc> algorithms;
std::map<std::string, AlgoCreateFunc> const &RPiController::getAlgorithms()
{
	return algorithms;
}

RegisterAlgorithm::RegisterAlgorithm(char const *name,
				     AlgoCreateFunc createFunc)
{
	algorithms[std::string(name)] = createFunc;
}
