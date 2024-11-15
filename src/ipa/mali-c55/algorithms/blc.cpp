/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Mali-C55 sensor offset (black level) correction
 */

#include "blc.h"

#include <libcamera/base/log.h>
#include <libcamera/control_ids.h>

#include "libcamera/internal/yaml_parser.h"

/**
 * \file blc.h
 */

namespace libcamera {

namespace ipa::mali_c55::algorithms {

/**
 * \class BlackLevelCorrection
 * \brief MaliC55 Black Level Correction control
 */

LOG_DEFINE_CATEGORY(MaliC55Blc)

BlackLevelCorrection::BlackLevelCorrection()
	: tuningParameters_(false)
{
}

/**
 * \copydoc libcamera::ipa::Algorithm::init
 */
int BlackLevelCorrection::init([[maybe_unused]] IPAContext &context,
			       const YamlObject &tuningData)
{
	offset00 = tuningData["offset00"].get<uint32_t>(0);
	offset01 = tuningData["offset01"].get<uint32_t>(0);
	offset10 = tuningData["offset10"].get<uint32_t>(0);
	offset11 = tuningData["offset11"].get<uint32_t>(0);

	if (offset00 > kMaxOffset || offset01 > kMaxOffset ||
	    offset10 > kMaxOffset || offset11 > kMaxOffset) {
		LOG(MaliC55Blc, Error) << "Invalid black level offsets";
		return -EINVAL;
	}

	tuningParameters_ = true;

	LOG(MaliC55Blc, Debug)
		<< "Black levels: 00 " << offset00 << ", 01 " << offset01
		<< ", 10 " << offset10 << ", 11 " << offset11;

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::configure
 */
int BlackLevelCorrection::configure(IPAContext &context,
				    [[maybe_unused]] const IPACameraSensorInfo &configInfo)
{
	/*
	 * If no Black Levels were passed in through tuning data then we could
	 * use the value from the CameraSensorHelper if one is available.
	 */
	if (context.configuration.sensor.blackLevel &&
	    !(offset00 + offset01 + offset10 + offset11)) {
		offset00 = context.configuration.sensor.blackLevel;
		offset01 = context.configuration.sensor.blackLevel;
		offset10 = context.configuration.sensor.blackLevel;
		offset11 = context.configuration.sensor.blackLevel;
	}

	return 0;
}

/**
 * \copydoc libcamera::ipa::Algorithm::prepare
 */
void BlackLevelCorrection::prepare([[maybe_unused]] IPAContext &context,
				   const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   mali_c55_params_buffer *params)
{
	mali_c55_params_block block;
	block.data = &params->data[params->total_size];

	if (frame > 0)
		return;

	if (!tuningParameters_)
		return;

	block.header->type = MALI_C55_PARAM_BLOCK_SENSOR_OFFS;
	block.header->flags = MALI_C55_PARAM_BLOCK_FL_NONE;
	block.header->size = sizeof(mali_c55_params_sensor_off_preshading);

	block.sensor_offs->chan00 = offset00;
	block.sensor_offs->chan01 = offset01;
	block.sensor_offs->chan10 = offset10;
	block.sensor_offs->chan11 = offset11;

	params->total_size += block.header->size;
}

void BlackLevelCorrection::process([[maybe_unused]] IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   [[maybe_unused]] const mali_c55_stats_buffer *stats,
				   ControlList &metadata)
{
	/*
	 * Black Level Offsets in tuning data need to be 20-bit, whereas the
	 * metadata expects values from a 16-bit range. Right-shift to remove
	 * the 4 least significant bits.
	 *
	 * The black levels should be reported in the order R, Gr, Gb, B. We
	 * ignore that here given we're using matching values so far, but it
	 * would be safer to check the sensor's bayer order.
	 *
	 * \todo Account for bayer order.
	 */
	metadata.set(controls::SensorBlackLevels, {
		static_cast<int32_t>(offset00 >> 4),
		static_cast<int32_t>(offset01 >> 4),
		static_cast<int32_t>(offset10 >> 4),
		static_cast<int32_t>(offset11 >> 4),
	});
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
