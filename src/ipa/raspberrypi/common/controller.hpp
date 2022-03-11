/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * controller.hpp - ISP controller interface
 */
#pragma once

// The Controller is simply a container for a collecting together a number of
// "control algorithms" (such as AWB etc.) and for running them all in a
// convenient manner.

#include <vector>
#include <string>

#include <linux/bcm2835-isp.h>

#include "camera_mode.h"
#include "device_status.h"
#include "metadata.hpp"

#include "pisp_statistics.h"

namespace RPiController {

class Algorithm;
using AlgorithmPtr = std::unique_ptr<Algorithm>;
using VC4StatisticsPtr = std::shared_ptr<bcm2835_isp_stats>;
using PiSPStatisticsPtr = std::shared_ptr<pisp_statistics>;


// The Controller holds a pointer to some global_metadata, which is how
// different controllers and control algorithms within them can exchange
// information. The Prepare function returns a pointer to metadata for this
// specific image, and which should be passed on to the Process function.

class Controller
{
public:
	Controller();
	Controller(char const *json_filename);
	virtual ~Controller() = 0;
	Algorithm *CreateAlgorithm(char const *name);
	void Read(char const *filename);
	void Initialise();
	void SwitchMode(CameraMode const &camera_mode, Metadata *metadata);
	Algorithm *GetAlgorithm(std::string const &name) const;

protected:
	std::vector<AlgorithmPtr> algorithms_;
	bool switch_mode_called_;
};

class VC4Controller : public Controller
{
public:
	VC4Controller()
		: Controller()
	{
	}

	VC4Controller(char const *json_filename)
		: Controller(json_filename)
	{
	}

	~VC4Controller() = default;
	void Prepare(Metadata *image_metadata);
	void Process(VC4StatisticsPtr stats, Metadata *image_metadata);
};

class PiSPController : public Controller
{
public:
	PiSPController()
		: Controller()
	{
	}

	PiSPController(char const *json_filename)
		: Controller(json_filename)
	{
	}

	~PiSPController() = default;
	void Prepare(PiSPStatisticsPtr stats, Metadata *image_metadata);
};


} // namespace RPiController
