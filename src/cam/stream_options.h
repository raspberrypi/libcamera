/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi (Trading) Ltd.
 *
 * stream_options.h - Helper to parse options for streams
 */
#ifndef __CAM_STREAM_OPTIONS_H__
#define __CAM_STREAM_OPTIONS_H__

#include <libcamera/camera.h>

#include "options.h"

using namespace libcamera;

class StreamKeyValueParser : public KeyValueParser
{
public:
	StreamKeyValueParser();

	KeyValueParser::Options parse(const char *arguments) override;

	static StreamRoles roles(const OptionValue &values);
	static int updateConfiguration(CameraConfiguration *config,
				       const OptionValue &values);

private:
	static bool parseRole(StreamRole *role,
			      const KeyValueParser::Options &options);
};

#endif /* __CAM_STREAM_OPTIONS_H__ */
