/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * focus.cpp - focus algorithm
 */
#include <stdint.h>

#include "libcamera/internal/log.h"

#include "../focus_status.h"
#include "focus.hpp"

using namespace RPi;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiFocus)

#define NAME "rpi.focus"

Focus::Focus(Controller *controller)
	: Algorithm(controller)
{
}

char const *Focus::Name() const
{
	return NAME;
}

void Focus::Process(StatisticsPtr &stats, Metadata *image_metadata)
{
	FocusStatus status;
	unsigned int i;
	for (i = 0; i < FOCUS_REGIONS; i++)
		status.focus_measures[i] = stats->focus_stats[i].contrast_val[1][1] / 1000;
	status.num = i;
	image_metadata->Set("focus.status", status);

	LOG(RPiFocus, Debug)
		<< "Focus contrast measure: "
		<< (status.focus_measures[5] + status.focus_measures[6]) / 10;
}

/* Register algorithm with the system. */
static Algorithm *Create(Controller *controller)
{
	return new Focus(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
