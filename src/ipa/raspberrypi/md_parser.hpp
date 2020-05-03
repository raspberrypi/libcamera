/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * md_parser.hpp - image sensor metadata parser interface
 */
#pragma once

#include <stdint.h>

/* Camera metadata parser class. Usage as shown below.

Setup:

Usually the metadata parser will be made as part of the CamHelper class so
application code doesn't have to worry which to kind to instantiate. But for
the sake of example let's suppose we're parsing imx219 metadata.

MdParser *parser = new MdParserImx219();  // for example
parser->SetBitsPerPixel(bpp);
parser->SetLineLengthBytes(pitch);
parser->SetNumLines(2);

Note 1: if you don't know how many lines there are, you can use SetBufferSize
instead to limit the total buffer size.

Note 2: if you don't know the line length, you can leave the line length unset
(or set to zero) and the parser will hunt for the line start instead. In this
case SetBufferSize *must* be used so that the parser won't run off the end of
the buffer.

Then on every frame:

if (parser->Parse(data) != MdParser::OK)
    much badness;
unsigned int exposure_lines, gain_code
if (parser->GetExposureLines(exposure_lines) != MdParser::OK)
    exposure was not found;
if (parser->GetGainCode(parser, gain_code) != MdParser::OK)
    gain code was not found;

(Note that the CamHelper class converts to/from exposure lines and time,
and gain_code / actual gain.)

If you suspect your embedded data may have changed its layout, change any line
lengths, number of lines, bits per pixel etc. that are different, and
then:

parser->Reset();

before calling Parse again. */

namespace RPi {

// Abstract base class from which other metadata parsers are derived.

class MdParser
{
public:
	// Parser status codes:
	// OK       - success
	// NOTFOUND - value such as exposure or gain was not found
	// ERROR    - all other errors
	enum Status {
		OK = 0,
		NOTFOUND = 1,
		ERROR = 2
	};
	MdParser() : reset_(true) {}
	virtual ~MdParser() {}
	void Reset() { reset_ = true; }
	void SetBitsPerPixel(int bpp) { bits_per_pixel_ = bpp; }
	void SetNumLines(unsigned int num_lines) { num_lines_ = num_lines; }
	void SetLineLengthBytes(unsigned int num_bytes)
	{
		line_length_bytes_ = num_bytes;
	}
	void SetBufferSize(unsigned int num_bytes)
	{
		buffer_size_bytes_ = num_bytes;
	}
	virtual Status Parse(void *data) = 0;
	virtual Status GetExposureLines(unsigned int &lines) = 0;
	virtual Status GetGainCode(unsigned int &gain_code) = 0;

protected:
	bool reset_;
	int bits_per_pixel_;
	unsigned int num_lines_;
	unsigned int line_length_bytes_;
	unsigned int buffer_size_bytes_;
};

// This isn't a full implementation of a metadata parser for SMIA sensors,
// however, it does provide the findRegs method which will prove useful and make
// it easier to implement parsers for other SMIA-like sensors (see
// md_parser_imx219.cpp for an example).

class MdParserSmia : public MdParser
{
public:
	MdParserSmia() : MdParser() {}

protected:
	// Note that error codes > 0 are regarded as non-fatal; codes < 0
	// indicate a bad data buffer. Status codes are:
	// PARSE_OK     - found all registers, much happiness
	// MISSING_REGS - some registers found; should this be a hard error?
	// The remaining codes are all hard errors.
	enum ParseStatus {
		PARSE_OK      =  0,
		MISSING_REGS  =  1,
		NO_LINE_START = -1,
		ILLEGAL_TAG   = -2,
		BAD_DUMMY     = -3,
		BAD_LINE_END  = -4,
		BAD_PADDING   = -5
	};
	ParseStatus findRegs(unsigned char *data, uint32_t regs[],
			     int offsets[], unsigned int num_regs);
};

} // namespace RPi
