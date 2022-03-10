/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * algorithm.cpp - ISP control algorithms
 */
#include <memory>

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

namespace {

/*
 * Wrap the algorithm map in a function to prevent static initialisation
 * failures caused by random ordering of static variable initialisation.
 * See https://isocpp.org/wiki/faq/ctors#static-init-order for more details.
 */
std::map<std::string, AlgoCreateFunc> *GetAlgorithmMap()
{
	static std::unique_ptr<std::map<std::string, AlgoCreateFunc>> algorithms =
			std::make_unique<std::map<std::string, AlgoCreateFunc>>();
	return algorithms.get();
}

} /* namespace */

std::map<std::string, AlgoCreateFunc> const &RPiController::GetAlgorithms()
{
	return *GetAlgorithmMap();
}

RegisterAlgorithm::RegisterAlgorithm(char const *name,
				     AlgoCreateFunc create_func)
{
	auto map = GetAlgorithmMap();
	map->emplace(name, create_func);
}
