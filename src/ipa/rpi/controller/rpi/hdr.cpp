/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022 Raspberry Pi Ltd
 *
 * hdr.cpp - HDR control algorithm
 */
#include "hdr.h"

#include <libcamera/base/log.h>

#include "hdr_status.h"

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiHdr)

#define NAME "rpi.hdr"

Hdr::Hdr(Controller *controller)
	: Algorithm(controller)
{
}

char const *Hdr::name() const
{
	return NAME;
}

int Hdr::read(const libcamera::YamlObject &params)
{
	config_.thresholdLo = params["threshold_lo"].get<uint16_t>(50000);
	config_.motionThreshold = params["motion_threshold"].get<double>(0.02);
	config_.diffPower = params["diff_power"].get<uint8_t>(13);
	if (config_.diffPower > 15) {
		LOG(RPiHdr, Error) << "Bad diff_power value";
		return -EINVAL;
	}
	return 0;
}

void Hdr::initialise()
{
}

void Hdr::prepare(Metadata *imageMetadata)
{
	HdrStatus hdrStatus;

	hdrStatus.diffPower = config_.diffPower;
	hdrStatus.motionThreshold = config_.motionThreshold;
	hdrStatus.thresholdLo = config_.thresholdLo;
	imageMetadata->set("hdr.status", hdrStatus);
}

// Register algorithm with the system.
static Algorithm *Create(Controller *controller)
{
	return (Algorithm *)new Hdr(controller);
}
static RegisterAlgorithm reg(NAME, &Create);
