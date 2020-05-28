/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * focus.cpp - focus algorithm
 */
#include <stdint.h>

#include "../focus_status.h"
#include "../logging.hpp"
#include "focus.hpp"

using namespace RPi;

#define NAME "rpi.focus"

Focus::Focus(Controller *controller)
	: Algorithm(controller)
{
}

char const *Focus::Name() const
{
	return NAME;
}

void Focus::Read(boost::property_tree::ptree const &params)
{
	print_ = params.get<int>("print", 0);
}

void Focus::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	FocusStatus status;
	unsigned int i;
	for (i = 0; i < FOCUS_REGIONS; i++)
		status.focus_measures[i] = stats->focus_stats[i].contrast_val[1][1] / 1000;
	status.num = i;
	image_metadata->Set("focus.status", status);
	if (print_) {
		uint32_t value = (status.focus_measures[5] + status.focus_measures[6]) / 10;
		RPI_LOG("Focus contrast measure: " << value);
	}
}

/* Register algorithm with the system. */
static Algorithm *Create(Controller *controller)
{
	return new Focus(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
