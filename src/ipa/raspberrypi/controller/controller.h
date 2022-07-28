/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * controller.h - ISP controller interface
 */
#pragma once

/*
 * The Controller is simply a container for a collecting together a number of
 * "control algorithms" (such as AWB etc.) and for running them all in a
 * convenient manner.
 */

#include <vector>
#include <string>

#include <linux/bcm2835-isp.h>

#include "libcamera/internal/yaml_parser.h"

#include "camera_mode.h"
#include "device_status.h"
#include "metadata.h"

namespace RPiController {

class Algorithm;
typedef std::unique_ptr<Algorithm> AlgorithmPtr;
typedef std::shared_ptr<bcm2835_isp_stats> StatisticsPtr;

/*
 * The Controller holds a pointer to some global_metadata, which is how
 * different controllers and control algorithms within them can exchange
 * information. The Prepare function returns a pointer to metadata for this
 * specific image, and which should be passed on to the Process function.
 */

class Controller
{
public:
	Controller();
	~Controller();
	int read(char const *filename);
	void initialise();
	void switchMode(CameraMode const &cameraMode, Metadata *metadata);
	void prepare(Metadata *imageMetadata);
	void process(StatisticsPtr stats, Metadata *imageMetadata);
	Metadata &getGlobalMetadata();
	Algorithm *getAlgorithm(std::string const &name) const;

protected:
	int createAlgorithm(const std::string &name, const libcamera::YamlObject &params);

	Metadata globalMetadata_;
	std::vector<AlgorithmPtr> algorithms_;
	bool switchModeCalled_;
};

} /* namespace RPiController */
