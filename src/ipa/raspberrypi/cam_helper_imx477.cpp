/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx477.cpp - camera helper for imx477 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#include "cam_helper.hpp"
#include "md_parser.hpp"

using namespace RPi;

/* Metadata parser implementation specific to Sony IMX477 sensors. */

class MdParserImx477 : public MdParserSmia
{
public:
	MdParserImx477();
	Status Parse(void *data) override;
	Status GetExposureLines(unsigned int &lines) override;
	Status GetGainCode(unsigned int &gain_code) override;
private:
	/* Offset of the register's value in the metadata block. */
	int reg_offsets_[4];
	/* Value of the register, once read from the metadata block. */
	int reg_values_[4];
};

class CamHelperImx477 : public CamHelper
{
public:
	CamHelperImx477();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	bool SensorEmbeddedDataPresent() const override;
	CamTransform GetOrientation() const override;
};

CamHelperImx477::CamHelperImx477()
	: CamHelper(new MdParserImx477())
{
}

uint32_t CamHelperImx477::GainCode(double gain) const
{
	return static_cast<uint32_t>(1024 - 1024 / gain);
}

double CamHelperImx477::Gain(uint32_t gain_code) const
{
	return 1024.0 / (1024 - gain_code);
}

bool CamHelperImx477::SensorEmbeddedDataPresent() const
{
	return true;
}

CamTransform CamHelperImx477::GetOrientation() const
{
	/* Camera is "upside down" on this board. */
	return CamTransform_HFLIP | CamTransform_VFLIP;
}

static CamHelper *Create()
{
	return new CamHelperImx477();
}

static RegisterCamHelper reg("imx477", &Create);

/*
 * We care about two gain registers and a pair of exposure registers. Their
 * I2C addresses from the Sony IMX477 datasheet:
 */
#define EXPHI_REG 0x0202
#define EXPLO_REG 0x0203
#define GAINHI_REG 0x0204
#define GAINLO_REG 0x0205

/*
 * Index of each into the reg_offsets and reg_values arrays. Must be in register
 * address order.
 */
#define EXPHI_INDEX 0
#define EXPLO_INDEX 1
#define GAINHI_INDEX 2
#define GAINLO_INDEX 3

MdParserImx477::MdParserImx477()
{
	reg_offsets_[0] = reg_offsets_[1] = reg_offsets_[2] = reg_offsets_[3] = -1;
}

MdParser::Status MdParserImx477::Parse(void *data)
{
	bool try_again = false;

	if (reset_) {
		/*
		 * Search again through the metadata for the gain and exposure
		 * registers.
		 */
		assert(bits_per_pixel_);
		assert(num_lines_ || buffer_size_bytes_);
		/* Need to be ordered */
		uint32_t regs[4] = {
			EXPHI_REG,
			EXPLO_REG,
			GAINHI_REG,
			GAINLO_REG
		};
		reg_offsets_[0] = reg_offsets_[1] = reg_offsets_[2] = reg_offsets_[3] = -1;
		int ret = static_cast<int>(findRegs(static_cast<uint8_t *>(data),
						    regs, reg_offsets_, 4));
		/*
		 * > 0 means "worked partially but parse again next time",
		 * < 0 means "hard error".
		 */
		if (ret > 0)
			try_again = true;
		else if (ret < 0)
			return ERROR;
	}

	for (int i = 0; i < 4; i++) {
		if (reg_offsets_[i] == -1)
			continue;

		reg_values_[i] = static_cast<uint8_t *>(data)[reg_offsets_[i]];
	}

	/* Re-parse next time if we were unhappy in some way. */
	reset_ = try_again;

	return OK;
}

MdParser::Status MdParserImx477::GetExposureLines(unsigned int &lines)
{
	if (reg_offsets_[EXPHI_INDEX] == -1 || reg_offsets_[EXPLO_INDEX] == -1)
		return NOTFOUND;

	lines = reg_values_[EXPHI_INDEX] * 256 + reg_values_[EXPLO_INDEX];

	return OK;
}

MdParser::Status MdParserImx477::GetGainCode(unsigned int &gain_code)
{
	if (reg_offsets_[GAINHI_INDEX] == -1 || reg_offsets_[GAINLO_INDEX] == -1)
		return NOTFOUND;

	gain_code = reg_values_[GAINHI_INDEX] * 256 + reg_values_[GAINLO_INDEX];

	return OK;
}
