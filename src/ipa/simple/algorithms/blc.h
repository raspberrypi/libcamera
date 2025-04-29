/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Red Hat Inc.
 *
 * Black level handling
 */

#pragma once

#include <optional>
#include <stdint.h>

#include "algorithm.h"

namespace libcamera {

namespace ipa::soft::algorithms {

class BlackLevel : public Algorithm
{
public:
	BlackLevel();
	~BlackLevel() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context, const IPAConfigInfo &configInfo) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const SwIspStats *stats,
		     ControlList &metadata) override;

private:
	std::optional<uint8_t> definedLevel_;
};

} /* namespace ipa::soft::algorithms */

} /* namespace libcamera */
