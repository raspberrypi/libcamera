/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * stream_options.cpp - Helper to parse options for streams
 */
#include "stream_options.h"

#include <iostream>

#include <libcamera/color_space.h>

using namespace libcamera;

StreamKeyValueParser::StreamKeyValueParser()
{
	addOption("role", OptionString,
		  "Role for the stream (viewfinder, video, still, raw)",
		  ArgumentRequired);
	addOption("width", OptionInteger, "Width in pixels",
		  ArgumentRequired);
	addOption("height", OptionInteger, "Height in pixels",
		  ArgumentRequired);
	addOption("pixelformat", OptionString, "Pixel format name",
		  ArgumentRequired);
	addOption("colorspace", OptionString, "Color space",
		  ArgumentRequired);
}

KeyValueParser::Options StreamKeyValueParser::parse(const char *arguments)
{
	KeyValueParser::Options options = KeyValueParser::parse(arguments);
	StreamRole role;

	if (options.valid() && options.isSet("role") &&
	    !parseRole(&role, options)) {
		std::cerr << "Unknown stream role "
			  << options["role"].toString() << std::endl;
		options.invalidate();
	}

	return options;
}

StreamRoles StreamKeyValueParser::roles(const OptionValue &values)
{
	/* If no configuration values to examine default to viewfinder. */
	if (values.empty())
		return { StreamRole::Viewfinder };

	const std::vector<OptionValue> &streamParameters = values.toArray();

	StreamRoles roles;
	for (auto const &value : streamParameters) {
		StreamRole role;

		/* If role is invalid or not set default to viewfinder. */
		if (!parseRole(&role, value.toKeyValues()))
			role = StreamRole::Viewfinder;

		roles.push_back(role);
	}

	return roles;
}

int StreamKeyValueParser::updateConfiguration(CameraConfiguration *config,
					      const OptionValue &values)
{
	if (!config) {
		std::cerr << "No configuration provided" << std::endl;
		return -EINVAL;
	}

	/* If no configuration values nothing to do. */
	if (values.empty())
		return 0;

	const std::vector<OptionValue> &streamParameters = values.toArray();

	if (config->size() != streamParameters.size()) {
		std::cerr
			<< "Number of streams in configuration "
			<< config->size()
			<< " does not match number of streams parsed "
			<< streamParameters.size()
			<< std::endl;
		return -EINVAL;
	}

	unsigned int i = 0;
	for (auto const &value : streamParameters) {
		KeyValueParser::Options opts = value.toKeyValues();
		StreamConfiguration &cfg = config->at(i++);

		if (opts.isSet("width") && opts.isSet("height")) {
			cfg.size.width = opts["width"];
			cfg.size.height = opts["height"];
		}

		if (opts.isSet("pixelformat"))
			cfg.pixelFormat = PixelFormat::fromString(opts["pixelformat"].toString());

		if (opts.isSet("colorspace"))
			cfg.colorSpace = ColorSpace::fromString(opts["colorspace"].toString());
	}

	return 0;
}

bool StreamKeyValueParser::parseRole(StreamRole *role,
				     const KeyValueParser::Options &options)
{
	if (!options.isSet("role"))
		return false;

	std::string name = options["role"].toString();

	if (name == "viewfinder") {
		*role = StreamRole::Viewfinder;
		return true;
	} else if (name == "video") {
		*role = StreamRole::VideoRecording;
		return true;
	} else if (name == "still") {
		*role = StreamRole::StillCapture;
		return true;
	} else if (name == "raw") {
		*role = StreamRole::Raw;
		return true;
	}

	return false;
}
