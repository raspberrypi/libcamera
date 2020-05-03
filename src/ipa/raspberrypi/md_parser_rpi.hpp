/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * md_parser_rpi.hpp - Raspberry Pi metadata parser interface
 */
#pragma once

#include "md_parser.hpp"

namespace RPi {

class MdParserRPi : public MdParser
{
public:
	MdParserRPi();
	Status Parse(void *data) override;
	Status GetExposureLines(unsigned int &lines) override;
	Status GetGainCode(unsigned int &gain_code) override;

private:
	// This must be the same struct that is filled into the metadata buffer
	// in the pipeline handler.
	struct rpiMetadata
	{
		uint32_t exposure;
		uint32_t gain;
	};
	rpiMetadata metadata;
};

}
