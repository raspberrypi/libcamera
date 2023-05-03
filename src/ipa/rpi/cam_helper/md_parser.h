/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * md_parser.h - image sensor metadata parser interface
 */
#pragma once

#include <initializer_list>
#include <map>
#include <optional>
#include <stdint.h>

#include <libcamera/base/span.h>

/*
 * Camera metadata parser class. Usage as shown below.
 *
 * Setup:
 *
 * Usually the metadata parser will be made as part of the CamHelper class so
 * application code doesn't have to worry which kind to instantiate. But for
 * the sake of example let's suppose we're parsing imx219 metadata.
 *
 * MdParser *parser = new MdParserSmia({ expHiReg, expLoReg, gainReg });
 * parser->SetBitsPerPixel(bpp);
 * parser->SetLineLengthBytes(pitch);
 * parser->SetNumLines(2);
 *
 * Note 1: if you don't know how many lines there are, the size of the input
 * buffer is used as a limit instead.
 *
 * Note 2: if you don't know the line length, you can leave the line length unset
 * (or set to zero) and the parser will hunt for the line start instead.
 *
 * Then on every frame:
 *
 * RegisterMap registers;
 * if (parser->Parse(buffer, registers) != MdParser::OK)
 *     much badness;
 * Metadata metadata;
 * CamHelper::PopulateMetadata(registers, metadata);
 *
 * (Note that the CamHelper class converts to/from exposure lines and time,
 * and gain_code / actual gain.)
 *
 * If you suspect your embedded data may have changed its layout, change any line
 * lengths, number of lines, bits per pixel etc. that are different, and
 * then:
 *
 * parser->Reset();
 *
 * before calling Parse again.
 */

namespace RPiController {

/* Abstract base class from which other metadata parsers are derived. */

class MdParser
{
public:
	using RegisterMap = std::map<uint32_t, uint32_t>;

	/*
	 * Parser status codes:
	 * OK       - success
	 * NOTFOUND - value such as exposure or gain was not found
	 * ERROR    - all other errors
	 */
	enum Status {
		OK = 0,
		NOTFOUND = 1,
		ERROR = 2
	};

	MdParser()
		: reset_(true), bitsPerPixel_(0), numLines_(0), lineLengthBytes_(0)
	{
	}

	virtual ~MdParser() = default;

	void reset()
	{
		reset_ = true;
	}

	void setBitsPerPixel(int bpp)
	{
		bitsPerPixel_ = bpp;
	}

	void setNumLines(unsigned int numLines)
	{
		numLines_ = numLines;
	}

	void setLineLengthBytes(unsigned int numBytes)
	{
		lineLengthBytes_ = numBytes;
	}

	virtual Status parse(libcamera::Span<const uint8_t> buffer,
			     RegisterMap &registers) = 0;

protected:
	bool reset_;
	int bitsPerPixel_;
	unsigned int numLines_;
	unsigned int lineLengthBytes_;
};

/*
 * This isn't a full implementation of a metadata parser for SMIA sensors,
 * however, it does provide the findRegs function which will prove useful and
 * make it easier to implement parsers for other SMIA-like sensors (see
 * md_parser_imx219.cpp for an example).
 */

class MdParserSmia final : public MdParser
{
public:
	MdParserSmia(std::initializer_list<uint32_t> registerList);

	MdParser::Status parse(libcamera::Span<const uint8_t> buffer,
			       RegisterMap &registers) override;

private:
	/* Maps register address to offset in the buffer. */
	using OffsetMap = std::map<uint32_t, std::optional<uint32_t>>;

	/*
	 * Note that error codes > 0 are regarded as non-fatal; codes < 0
	 * indicate a bad data buffer. Status codes are:
	 * ParseOk     - found all registers, much happiness
	 * MissingRegs - some registers found; should this be a hard error?
	 * The remaining codes are all hard errors.
	 */
	enum ParseStatus {
		ParseOk      =  0,
		MissingRegs  =  1,
		NoLineStart  = -1,
		IllegalTag   = -2,
		BadDummy     = -3,
		BadLineEnd   = -4,
		BadPadding   = -5
	};

	ParseStatus findRegs(libcamera::Span<const uint8_t> buffer);

	OffsetMap offsets_;
};

} /* namespace RPi */
