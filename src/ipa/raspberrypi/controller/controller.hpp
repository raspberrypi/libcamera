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

namespace RPi {

class Algorithm;
typedef std::unique_ptr<Algorithm> AlgorithmPtr;
typedef std::shared_ptr<bcm2835_isp_stats> StatisticsPtr;

// The Controller holds a pointer to some global_metadata, which is how
// different controllers and control algorithms within them can exchange
// information. The Prepare method returns a pointer to metadata for this
// specific image, and which should be passed on to the Process method.

class Controller
{
public:
	Controller();
	Controller(char const *json_filename);
	~Controller();
	Algorithm *CreateAlgorithm(char const *name);
	void Read(char const *filename);
	void Initialise();
	void SwitchMode(CameraMode const &camera_mode);
	void Prepare(Metadata *image_metadata);
	void Process(StatisticsPtr stats, Metadata *image_metadata);
	Metadata &GetGlobalMetadata();
	Algorithm *GetAlgorithm(std::string const &name) const;

protected:
	Metadata global_metadata_;
	std::vector<AlgorithmPtr> algorithms_;
	bool switch_mode_called_;
};

} // namespace RPi
