/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * capture_script.h - Capture session configuration script
 */

#pragma once

#include <map>
#include <memory>
#include <string>

#include <libcamera/camera.h>
#include <libcamera/controls.h>

#include <yaml.h>

class CaptureScript
{
public:
	CaptureScript(std::shared_ptr<libcamera::Camera> camera,
		      const std::string &fileName);

	bool valid() const { return valid_; }

	const libcamera::ControlList &frameControls(unsigned int frame);

private:
	struct EventDeleter {
		void operator()(yaml_event_t *event) const
		{
			yaml_event_delete(event);
			delete event;
		}
	};
	using EventPtr = std::unique_ptr<yaml_event_t, EventDeleter>;

	std::map<std::string, const libcamera::ControlId *> controls_;
	std::map<unsigned int, libcamera::ControlList> frameControls_;
	std::shared_ptr<libcamera::Camera> camera_;
	yaml_parser_t parser_;
	unsigned int loop_;
	bool valid_;

	EventPtr nextEvent(yaml_event_type_t expectedType = YAML_NO_EVENT);
	bool checkEvent(const EventPtr &event, yaml_event_type_t expectedType) const;
	static std::string eventScalarValue(const EventPtr &event);
	static std::string eventTypeName(yaml_event_type_t type);

	int parseScript(FILE *script);

	int parseProperties();
	int parseProperty();
	int parseFrames();
	int parseFrame(EventPtr event);
	int parseControl(EventPtr event, libcamera::ControlList &controls);

	libcamera::ControlValue parseScalarControl(const libcamera::ControlId *id,
						   const std::string repr);
	libcamera::ControlValue parseArrayControl(const libcamera::ControlId *id,
						  const std::vector<std::string> &repr);

	std::string parseScalar();
	libcamera::ControlValue parseRectangles();
	std::vector<std::vector<std::string>> parseArrays();
	std::vector<std::string> parseSingleArray();

	void unpackFailure(const libcamera::ControlId *id,
			   const std::string &repr);
	libcamera::ControlValue unpackControl(const libcamera::ControlId *id);
	libcamera::Rectangle unpackRectangle(const std::vector<std::string> &strVec);
};
