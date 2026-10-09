/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026, Ideas On Board
 *
 * Table-based Lsc implementation
 */
#pragma once

#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/value_node.h"

#include "lsc_base.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(LscTable)

namespace ipa {

class LscTable : public LscImplementation
{
public:
	int parseLscData(const ValueNode &sets,
			 const LscDescriptor &descriptor) override;

	LscImplementation::ComponentsMap
	sampleForCrop([[maybe_unused]] const Rectangle &cropRectangle,
		      [[maybe_unused]] std::vector<double> xPos,
		      [[maybe_unused]] std::vector<double> yPos) override
	{
		LOG(LscTable, Warning)
			<< "Tabular LSC data doesn't support resampling";
		return lscData_;
	}

private:
	int parseLscComponent(const ValueNode &yamlSet,
			      unsigned int ct, const LscDescriptor &descriptor);
	std::vector<float> parseTable(const ValueNode &tuningData,
					 const char *prop,
					 unsigned int numHSamples,
					 unsigned int numVSamples);

	LscImplementation::ComponentsMap lscData_;
};

} /* namespace ipa */

} /* namespace libcamera */
