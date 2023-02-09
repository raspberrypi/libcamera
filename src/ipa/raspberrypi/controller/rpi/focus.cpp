/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * focus.cpp - focus algorithm
 */
#include <stdint.h>

#include <libcamera/base/log.h>

#include "../focus_status.h"
#include "focus.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiFocus)

#define NAME "rpi.focus"

Focus::Focus(Controller *controller)
	: Algorithm(controller)
{
}

char const *Focus::name() const
{
	return NAME;
}

void Focus::process(StatisticsPtr &stats, Metadata *imageMetadata)
{
	FocusStatus status;
	for (unsigned int i = 0; i < stats->focusRegions.numRegions(); i++)
		status.focusMeasures[i] = stats->focusRegions.get(i).val;
	status.num = stats->focusRegions.numRegions();
	imageMetadata->set("focus.status", status);

	LOG(RPiFocus, Debug)
		<< "Focus contrast measure: "
		<< (status.focusMeasures[5] + status.focusMeasures[6]) / 10;
}

/* Register algorithm with the system. */
static Algorithm *create(Controller *controller)
{
	return new Focus(controller);
}
static RegisterAlgorithm reg(NAME, &create);
