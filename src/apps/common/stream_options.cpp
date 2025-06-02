/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Raspberry Pi Ltd
 *
 * Helper to parse options for streams
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

	if (options.valid() && options.isSet("role") && !parseRole(options)) {
		std::cerr << "Unknown stream role "
			  << options["role"].toString() << std::endl;
		options.invalidate();
	}

	return options;
}

std::vector<StreamRole> StreamKeyValueParser::roles(const OptionValue &values)
{
	if (values.empty())
		return {};

	const std::vector<OptionValue> &streamParameters = values.toArray();

	std::vector<StreamRole> roles;
	for (auto const &value : streamParameters) {
		/* If a role is invalid default it to viewfinder. */
		roles.push_back(parseRole(value.toKeyValues()).value_or(StreamRole::Viewfinder));
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

std::optional<libcamera::StreamRole> StreamKeyValueParser::parseRole(const KeyValueParser::Options &options)
{
	if (!options.isSet("role"))
		return {};

	std::string name = options["role"].toString();

	if (name == "viewfinder")
		return StreamRole::Viewfinder;
	else if (name == "video")
		return StreamRole::VideoRecording;
	else if (name == "still")
		return StreamRole::StillCapture;
	else if (name == "raw")
		return StreamRole::Raw;

	return {};
}
