/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * cam_helper_imx219.cpp - camera helper for imx219 sensor
 */

#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * We have observed the imx219 embedded data stream randomly return junk
 * reister values.  Do not rely on embedded data until this has been resolved.
 */
#define ENABLE_EMBEDDED_DATA 0

#include "cam_helper.hpp"
#if ENABLE_EMBEDDED_DATA
#include "md_parser.hpp"
#else
#include "md_parser_rpi.hpp"
#endif

using namespace RPi;

/* Metadata parser implementation specific to Sony IMX219 sensors. */

class MdParserImx219 : public MdParserSmia
{
public:
	MdParserImx219();
	Status Parse(void *data) override;
	Status GetExposureLines(unsigned int &lines) override;
	Status GetGainCode(unsigned int &gain_code) override;
private:
	/* Offset of the register's value in the metadata block. */
	int reg_offsets_[3];
	/* Value of the register, once read from the metadata block. */
	int reg_values_[3];
};

class CamHelperImx219 : public CamHelper
{
public:
	CamHelperImx219();
	uint32_t GainCode(double gain) const override;
	double Gain(uint32_t gain_code) const override;
	unsigned int MistrustFramesModeSwitch() const override;
	bool SensorEmbeddedDataPresent() const override;
	CamTransform GetOrientation() const override;
};

CamHelperImx219::CamHelperImx219()
#if ENABLE_EMBEDDED_DATA
	: CamHelper(new MdParserImx219())
#else
	: CamHelper(new MdParserRPi())
#endif
{
}

uint32_t CamHelperImx219::GainCode(double gain) const
{
	return (uint32_t)(256 - 256 / gain);
}

double CamHelperImx219::Gain(uint32_t gain_code) const
{
	return 256.0 / (256 - gain_code);
}

unsigned int CamHelperImx219::MistrustFramesModeSwitch() const
{
	/*
	 * For reasons unknown, we do occasionally get a bogus metadata frame
	 * at a mode switch (though not at start-up). Possibly warrants some
	 * investigation, though not a big deal.
	 */
	return 1;
}

bool CamHelperImx219::SensorEmbeddedDataPresent() const
{
	return ENABLE_EMBEDDED_DATA;
}

CamTransform CamHelperImx219::GetOrientation() const
{
	/* Camera is "upside down" on this board. */
	return CamTransform_HFLIP | CamTransform_VFLIP;
}

static CamHelper *Create()
{
	return new CamHelperImx219();
}

static RegisterCamHelper reg("imx219", &Create);

/*
 * We care about one gain register and a pair of exposure registers. Their I2C
 * addresses from the Sony IMX219 datasheet:
 */
#define GAIN_REG 0x157
#define EXPHI_REG 0x15A
#define EXPLO_REG 0x15B

/*
 * Index of each into the reg_offsets and reg_values arrays. Must be in
 * register address order.
 */
#define GAIN_INDEX 0
#define EXPHI_INDEX 1
#define EXPLO_INDEX 2

MdParserImx219::MdParserImx219()
{
	reg_offsets_[0] = reg_offsets_[1] = reg_offsets_[2] = -1;
}

MdParser::Status MdParserImx219::Parse(void *data)
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
		uint32_t regs[3] = { GAIN_REG, EXPHI_REG, EXPLO_REG };
		reg_offsets_[0] = reg_offsets_[1] = reg_offsets_[2] = -1;
		int ret = static_cast<int>(findRegs(static_cast<uint8_t *>(data),
						    regs, reg_offsets_, 3));
		/*
		 * > 0 means "worked partially but parse again next time",
		 * < 0 means "hard error".
		 */
		if (ret > 0)
			try_again = true;
		else if (ret < 0)
			return ERROR;
	}

	for (int i = 0; i < 3; i++) {
		if (reg_offsets_[i] == -1)
			continue;

		reg_values_[i] = static_cast<uint8_t *>(data)[reg_offsets_[i]];
	}

	/* Re-parse next time if we were unhappy in some way. */
	reset_ = try_again;

	return OK;
}

MdParser::Status MdParserImx219::GetExposureLines(unsigned int &lines)
{
	if (reg_offsets_[EXPHI_INDEX] == -1 || reg_offsets_[EXPLO_INDEX] == -1)
		return NOTFOUND;

	lines = reg_values_[EXPHI_INDEX] * 256 + reg_values_[EXPLO_INDEX];

	return OK;
}

MdParser::Status MdParserImx219::GetGainCode(unsigned int &gain_code)
{
	if (reg_offsets_[GAIN_INDEX] == -1)
		return NOTFOUND;

	gain_code = reg_values_[GAIN_INDEX];

	return OK;
}
