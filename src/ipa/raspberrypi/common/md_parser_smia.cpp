/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019-2021, Raspberry Pi (Trading) Limited
 *
 * md_parser_smia.cpp - SMIA specification based embedded data parser
 */

#include <libcamera/base/log.h>
#include "md_parser.hpp"

using namespace RPiController;
using namespace libcamera;

/*
 * This function goes through the embedded data to find the offsets (not
 * values!), in the data block, where the values of the given registers can
 * subsequently be found.
 *
 * Embedded data tag bytes, from Sony IMX219 datasheet but general to all SMIA
 * sensors, I think.
 */

constexpr unsigned int LINE_START = 0x0a;
constexpr unsigned int LINE_END_TAG = 0x07;
constexpr unsigned int REG_HI_BITS = 0xaa;
constexpr unsigned int REG_LOW_BITS = 0xa5;
constexpr unsigned int REG_VALUE = 0x5a;
constexpr unsigned int REG_SKIP = 0x55;

MdParserSmia::MdParserSmia(std::initializer_list<uint32_t> registerList)
{
	for (auto r : registerList)
		offsets_[r] = {};
}

MdParser::Status MdParserSmia::Parse(libcamera::Span<const uint8_t> buffer,
				     RegisterMap &registers)
{
	if (reset_) {
		/*
		 * Search again through the metadata for all the registers
		 * requested.
		 */
		ASSERT(bits_per_pixel_);

		for (const auto &kv : offsets_)
			offsets_[kv.first] = {};

		ParseStatus ret = findRegs(buffer);
		/*
		 * > 0 means "worked partially but parse again next time",
		 * < 0 means "hard error".
		 *
		 * In either case, we retry parsing on the next frame.
		 */
		if (ret != PARSE_OK)
			return ERROR;

		reset_ = false;
	}

	/* Populate the register values requested. */
	registers.clear();
	for (const auto &[reg, offset] : offsets_) {
		if (!offset) {
			reset_ = true;
			return NOTFOUND;
		}
		registers[reg] = buffer[offset.value()];
	}

	return OK;
}

MdParserSmia::ParseStatus MdParserSmia::findRegs(libcamera::Span<const uint8_t> buffer)
{
	ASSERT(offsets_.size());

	if (buffer[0] != LINE_START)
		return NO_LINE_START;

	unsigned int current_offset = 1; /* after the LINE_START */
	unsigned int current_line_start = 0, current_line = 0;
	unsigned int reg_num = 0, regs_done = 0;

	while (1) {
		int tag = buffer[current_offset++];

		if ((bits_per_pixel_ == 10 &&
		     (current_offset + 1 - current_line_start) % 5 == 0) ||
		    (bits_per_pixel_ == 12 &&
		     (current_offset + 1 - current_line_start) % 3 == 0)) {
			if (buffer[current_offset++] != REG_SKIP)
				return BAD_DUMMY;
		}

		int data_byte = buffer[current_offset++];

		if (tag == LINE_END_TAG) {
			if (data_byte != LINE_END_TAG)
				return BAD_LINE_END;

			if (num_lines_ && ++current_line == num_lines_)
				return MISSING_REGS;

			if (line_length_bytes_) {
				current_offset = current_line_start + line_length_bytes_;

				/* Require whole line to be in the buffer (if buffer size set). */
				if (buffer.size() &&
				    current_offset + line_length_bytes_ > buffer.size())
					return MISSING_REGS;

				if (buffer[current_offset] != LINE_START)
					return NO_LINE_START;
			} else {
				/* allow a zero line length to mean "hunt for the next line" */
				while (current_offset < buffer.size() &&
				       buffer[current_offset] != LINE_START)
					current_offset++;

				if (current_offset == buffer.size())
					return NO_LINE_START;
			}

			/* inc current_offset to after LINE_START */
			current_line_start = current_offset++;
		} else {
			if (tag == REG_HI_BITS)
				reg_num = (reg_num & 0xff) | (data_byte << 8);
			else if (tag == REG_LOW_BITS)
				reg_num = (reg_num & 0xff00) | data_byte;
			else if (tag == REG_SKIP)
				reg_num++;
			else if (tag == REG_VALUE) {
				auto reg = offsets_.find(reg_num);

				if (reg != offsets_.end()) {
					offsets_[reg_num] = current_offset - 1;

					if (++regs_done == offsets_.size())
						return PARSE_OK;
				}
				reg_num++;
			} else
				return ILLEGAL_TAG;
		}
	}
}
