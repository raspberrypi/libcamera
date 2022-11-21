/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * algorithm.h - ISP control algorithm interface
 */
#pragma once

/*
 * All algorithms should be derived from this class and made available to the
 * Controller.
 */

#include <string>
#include <memory>
#include <map>

#include "libcamera/internal/yaml_parser.h"

#include "controller.h"

namespace RPiController {

/* This defines the basic interface for all control algorithms. */

class Algorithm
{
public:
	Algorithm(Controller *controller)
		: controller_(controller)
	{
	}
	virtual ~Algorithm() = default;
	virtual char const *name() const = 0;
	virtual int read(const libcamera::YamlObject &params);
	virtual void initialise();
	virtual void switchMode(CameraMode const &cameraMode, Metadata *metadata);
	virtual void prepare(Metadata *imageMetadata);
	virtual void process(StatisticsPtr &stats, Metadata *imageMetadata);
	Metadata &getGlobalMetadata() const
	{
		return controller_->getGlobalMetadata();
	}

private:
	Controller *controller_;
};

/*
 * This code is for automatic registration of Front End algorithms with the
 * system.
 */

typedef Algorithm *(*AlgoCreateFunc)(Controller *controller);
struct RegisterAlgorithm {
	RegisterAlgorithm(char const *name, AlgoCreateFunc createFunc);
};
std::map<std::string, AlgoCreateFunc> const &getAlgorithms();

} /* namespace RPiController */
