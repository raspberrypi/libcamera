/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * lux.h - RkISP1 Lux control
 */

#pragma once

#include <sys/types.h>

#include "libipa/lux.h"

#include "algorithm.h"

namespace libcamera {

namespace ipa::rkisp1::algorithms {

class Lux : public Algorithm
{
public:
	Lux();

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const rkisp1_stat_buffer *stats,
		     ControlList &metadata) override;

private:
	ipa::Lux lux_;
};

} /* namespace ipa::rkisp1::algorithms */
} /* namespace libcamera */
