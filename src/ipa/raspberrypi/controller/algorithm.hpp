/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * algorithm.hpp - ISP control algorithm interface
 */
#pragma once

// All algorithms should be derived from this class and made available to the
// Controller.

#include <string>
#include <memory>
#include <map>
#include <atomic>

#include "logging.hpp"
#include "controller.hpp"

#include <boost/property_tree/ptree.hpp>

namespace RPi {

// This defines the basic interface for all control algorithms.

class Algorithm
{
public:
	Algorithm(Controller *controller)
		: controller_(controller), paused_(false)
	{
	}
	virtual ~Algorithm() {}
	virtual char const *Name() const = 0;
	virtual bool IsPaused() const { return paused_; }
	virtual void Pause() { paused_ = true; }
	virtual void Resume() { paused_ = false; }
	virtual void Read(boost::property_tree::ptree const &params);
	virtual void Initialise();
	virtual void SwitchMode(CameraMode const &camera_mode);
	virtual void Prepare(Metadata *image_metadata);
	virtual void Process(StatisticsPtr &stats, Metadata *image_metadata);
	Metadata &GetGlobalMetadata() const
	{
		return controller_->GetGlobalMetadata();
	}

private:
	Controller *controller_;
	std::atomic<bool> paused_;
};

// This code is for automatic registration of Front End algorithms with the
// system.

typedef Algorithm *(*AlgoCreateFunc)(Controller *controller);
struct RegisterAlgorithm {
	RegisterAlgorithm(char const *name, AlgoCreateFunc create_func);
};
std::map<std::string, AlgoCreateFunc> const &GetAlgorithms();

} // namespace RPi
