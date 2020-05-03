/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * md_parser_rpi.cpp - Metadata parser for generic Raspberry Pi metadata
 */

#include <string.h>

#include "md_parser_rpi.hpp"

using namespace RPi;

MdParserRPi::MdParserRPi()
{
}

MdParser::Status MdParserRPi::Parse(void *data)
{
	if (buffer_size_bytes_ < sizeof(rpiMetadata))
		return ERROR;

	memcpy(&metadata, data, sizeof(rpiMetadata));
	return OK;
}

MdParser::Status MdParserRPi::GetExposureLines(unsigned int &lines)
{
	lines = metadata.exposure;
	return OK;
}

MdParser::Status MdParserRPi::GetGainCode(unsigned int &gain_code)
{
	gain_code = metadata.gain;
	return OK;
}
