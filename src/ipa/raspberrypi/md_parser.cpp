/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * md_parser.cpp - image sensor metadata parsers
 */

#include <assert.h>
#include <map>
#include <string.h>

#include "md_parser.hpp"

using namespace RPi;

// This function goes through the embedded data to find the offsets (not
// values!), in the data block, where the values of the given registers can
// subsequently be found.

// Embedded data tag bytes, from Sony IMX219 datasheet but general to all SMIA
// sensors, I think.

#define LINE_START 0x0a
#define LINE_END_TAG 0x07
#define REG_HI_BITS 0xaa
#define REG_LOW_BITS 0xa5
#define REG_VALUE 0x5a
#define REG_SKIP 0x55

MdParserSmia::ParseStatus MdParserSmia::findRegs(unsigned char *data,
						 uint32_t regs[], int offsets[],
						 unsigned int num_regs)
{
	assert(num_regs > 0);
	if (data[0] != LINE_START)
		return NO_LINE_START;

	unsigned int current_offset = 1; // after the LINE_START
	unsigned int current_line_start = 0, current_line = 0;
	unsigned int reg_num = 0, first_reg = 0;
	ParseStatus retcode = PARSE_OK;
	while (1) {
		int tag = data[current_offset++];
		if ((bits_per_pixel_ == 10 &&
		     (current_offset + 1 - current_line_start) % 5 == 0) ||
		    (bits_per_pixel_ == 12 &&
		     (current_offset + 1 - current_line_start) % 3 == 0)) {
			if (data[current_offset++] != REG_SKIP)
				return BAD_DUMMY;
		}
		int data_byte = data[current_offset++];
		//printf("Offset %u, tag 0x%02x data_byte 0x%02x\n", current_offset-1, tag, data_byte);
		if (tag == LINE_END_TAG) {
			if (data_byte != LINE_END_TAG)
				return BAD_LINE_END;
			if (num_lines_ && ++current_line == num_lines_)
				return MISSING_REGS;
			if (line_length_bytes_) {
				current_offset =
					current_line_start + line_length_bytes_;
				// Require whole line to be in the buffer (if buffer size set).
				if (buffer_size_bytes_ &&
				    current_offset + line_length_bytes_ >
					    buffer_size_bytes_)
					return MISSING_REGS;
				if (data[current_offset] != LINE_START)
					return NO_LINE_START;
			} else {
				// allow a zero line length to mean "hunt for the next line"
				while (data[current_offset] != LINE_START &&
				       current_offset < buffer_size_bytes_)
					current_offset++;
				if (current_offset == buffer_size_bytes_)
					return NO_LINE_START;
			}
			// inc current_offset to after LINE_START
			current_line_start =
				current_offset++;
		} else {
			if (tag == REG_HI_BITS)
				reg_num = (reg_num & 0xff) | (data_byte << 8);
			else if (tag == REG_LOW_BITS)
				reg_num = (reg_num & 0xff00) | data_byte;
			else if (tag == REG_SKIP)
				reg_num++;
			else if (tag == REG_VALUE) {
				while (reg_num >=
				       // assumes registers are in order...
				       regs[first_reg]) {
					if (reg_num == regs[first_reg])
						offsets[first_reg] =
							current_offset - 1;
					if (++first_reg == num_regs)
						return retcode;
				}
				reg_num++;
			} else
				return ILLEGAL_TAG;
		}
	}
}
