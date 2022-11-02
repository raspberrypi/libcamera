/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Ideas on Board Oy
 *
 * capture_script.cpp - Capture session configuration script
 */

#include "capture_script.h"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace libcamera;

CaptureScript::CaptureScript(std::shared_ptr<Camera> camera,
			     const std::string &fileName)
	: camera_(camera), loop_(0), valid_(false)
{
	FILE *fh = fopen(fileName.c_str(), "r");
	if (!fh) {
		int ret = -errno;
		std::cerr << "Failed to open capture script " << fileName
			  << ": " << strerror(-ret) << std::endl;
		return;
	}

	/*
	 * Map the camera's controls to their name so that they can be
	 * easily identified when parsing the script file.
	 */
	for (const auto &[control, info] : camera_->controls())
		controls_[control->name()] = control;

	int ret = parseScript(fh);
	fclose(fh);
	if (ret)
		return;

	valid_ = true;
}

/* Retrieve the control list associated with a frame number. */
const ControlList &CaptureScript::frameControls(unsigned int frame)
{
	static ControlList controls{};
	unsigned int idx = frame;

	/* If we loop, repeat the controls every 'loop_' frames. */
	if (loop_)
		idx = frame % loop_;

	auto it = frameControls_.find(idx);
	if (it == frameControls_.end())
		return controls;

	return it->second;
}

CaptureScript::EventPtr CaptureScript::nextEvent(yaml_event_type_t expectedType)
{
	EventPtr event(new yaml_event_t);

	if (!yaml_parser_parse(&parser_, event.get()))
		return nullptr;

	if (expectedType != YAML_NO_EVENT && !checkEvent(event, expectedType))
		return nullptr;

	return event;
}

bool CaptureScript::checkEvent(const EventPtr &event, yaml_event_type_t expectedType) const
{
	if (event->type != expectedType) {
		std::cerr << "Capture script error on line " << event->start_mark.line
			  << " column " << event->start_mark.column << ": "
			  << "Expected " << eventTypeName(expectedType)
			  << " event, got " << eventTypeName(event->type)
			  << std::endl;
		return false;
	}

	return true;
}

std::string CaptureScript::eventScalarValue(const EventPtr &event)
{
	return std::string(reinterpret_cast<char *>(event->data.scalar.value),
			   event->data.scalar.length);
}

std::string CaptureScript::eventTypeName(yaml_event_type_t type)
{
	static const std::map<yaml_event_type_t, std::string> typeNames = {
		{ YAML_STREAM_START_EVENT, "stream-start" },
		{ YAML_STREAM_END_EVENT, "stream-end" },
		{ YAML_DOCUMENT_START_EVENT, "document-start" },
		{ YAML_DOCUMENT_END_EVENT, "document-end" },
		{ YAML_ALIAS_EVENT, "alias" },
		{ YAML_SCALAR_EVENT, "scalar" },
		{ YAML_SEQUENCE_START_EVENT, "sequence-start" },
		{ YAML_SEQUENCE_END_EVENT, "sequence-end" },
		{ YAML_MAPPING_START_EVENT, "mapping-start" },
		{ YAML_MAPPING_END_EVENT, "mapping-end" },
	};

	auto it = typeNames.find(type);
	if (it == typeNames.end())
		return "[type " + std::to_string(type) + "]";

	return it->second;
}

int CaptureScript::parseScript(FILE *script)
{
	int ret = yaml_parser_initialize(&parser_);
	if (!ret) {
		std::cerr << "Failed to initialize yaml parser" << std::endl;
		return ret;
	}

	/* Delete the parser upon function exit. */
	struct ParserDeleter {
		ParserDeleter(yaml_parser_t *parser) : parser_(parser) { }
		~ParserDeleter() { yaml_parser_delete(parser_); }
		yaml_parser_t *parser_;
	} deleter(&parser_);

	yaml_parser_set_input_file(&parser_, script);

	EventPtr event = nextEvent(YAML_STREAM_START_EVENT);
	if (!event)
		return -EINVAL;

	event = nextEvent(YAML_DOCUMENT_START_EVENT);
	if (!event)
		return -EINVAL;

	event = nextEvent(YAML_MAPPING_START_EVENT);
	if (!event)
		return -EINVAL;

	while (1) {
		event = nextEvent();
		if (!event)
			return -EINVAL;

		if (event->type == YAML_MAPPING_END_EVENT)
			return 0;

		if (!checkEvent(event, YAML_SCALAR_EVENT))
			return -EINVAL;

		std::string section = eventScalarValue(event);

		if (section == "properties") {
			ret = parseProperties();
			if (ret)
				return ret;
		} else if (section == "frames") {
			ret = parseFrames();
			if (ret)
				return ret;
		} else {
			std::cerr << "Unsupported section '" << section << "'"
				  << std::endl;
			return -EINVAL;
		}
	}
}

int CaptureScript::parseProperty()
{
	EventPtr event = nextEvent(YAML_MAPPING_START_EVENT);
	if (!event)
		return -EINVAL;

	std::string prop = parseScalar();
	if (prop.empty())
		return -EINVAL;

	if (prop == "loop") {
		event = nextEvent();
		if (!event)
			return -EINVAL;

		std::string value = eventScalarValue(event);
		if (value.empty())
			return -EINVAL;

		loop_ = atoi(value.c_str());
		if (!loop_) {
			std::cerr << "Invalid loop limit '" << loop_ << "'"
				  << std::endl;
			return -EINVAL;
		}
	} else {
		std::cerr << "Unsupported property '" << prop << "'" << std::endl;
		return -EINVAL;
	}

	event = nextEvent(YAML_MAPPING_END_EVENT);
	if (!event)
		return -EINVAL;

	return 0;
}

int CaptureScript::parseProperties()
{
	EventPtr event = nextEvent(YAML_SEQUENCE_START_EVENT);
	if (!event)
		return -EINVAL;

	while (1) {
		if (event->type == YAML_SEQUENCE_END_EVENT)
			return 0;

		int ret = parseProperty();
		if (ret)
			return ret;

		event = nextEvent();
		if (!event)
			return -EINVAL;
	}

	return 0;
}

int CaptureScript::parseFrames()
{
	EventPtr event = nextEvent(YAML_SEQUENCE_START_EVENT);
	if (!event)
		return -EINVAL;

	while (1) {
		event = nextEvent();
		if (!event)
			return -EINVAL;

		if (event->type == YAML_SEQUENCE_END_EVENT)
			return 0;

		int ret = parseFrame(std::move(event));
		if (ret)
			return ret;
	}
}

int CaptureScript::parseFrame(EventPtr event)
{
	if (!checkEvent(event, YAML_MAPPING_START_EVENT))
		return -EINVAL;

	std::string key = parseScalar();
	if (key.empty())
		return -EINVAL;

	unsigned int frameId = atoi(key.c_str());
	if (loop_ && frameId >= loop_) {
		std::cerr
			<< "Frame id (" << frameId << ") shall be smaller than"
			<< "loop limit (" << loop_ << ")" << std::endl;
		return -EINVAL;
	}

	event = nextEvent(YAML_MAPPING_START_EVENT);
	if (!event)
		return -EINVAL;

	ControlList controls{};

	while (1) {
		event = nextEvent();
		if (!event)
			return -EINVAL;

		if (event->type == YAML_MAPPING_END_EVENT)
			break;

		int ret = parseControl(std::move(event), controls);
		if (ret)
			return ret;
	}

	frameControls_[frameId] = std::move(controls);

	event = nextEvent(YAML_MAPPING_END_EVENT);
	if (!event)
		return -EINVAL;

	return 0;
}

int CaptureScript::parseControl(EventPtr event, ControlList &controls)
{
	/* We expect a value after a key. */
	std::string name = eventScalarValue(event);
	if (name.empty())
		return -EINVAL;

	/* If the camera does not support the control just ignore it. */
	auto it = controls_.find(name);
	if (it == controls_.end()) {
		std::cerr << "Unsupported control '" << name << "'" << std::endl;
		return -EINVAL;
	}

	const ControlId *controlId = it->second;

	ControlValue val = unpackControl(controlId);
	if (val.isNone()) {
		std::cerr << "Error unpacking control '" << name << "'"
			  << std::endl;
		return -EINVAL;
	}

	controls.set(controlId->id(), val);

	return 0;
}

std::string CaptureScript::parseScalar()
{
	EventPtr event = nextEvent(YAML_SCALAR_EVENT);
	if (!event)
		return "";

	return eventScalarValue(event);
}

ControlValue CaptureScript::parseRectangles()
{
	std::vector<libcamera::Rectangle> rectangles;

	std::vector<std::vector<std::string>> arrays = parseArrays();
	if (arrays.empty())
		return {};

	for (const std::vector<std::string> &values : arrays) {
		if (values.size() != 4) {
			std::cerr << "Error parsing Rectangle: expected "
				  << "array with 4 parameters" << std::endl;
			return {};
		}

		Rectangle rect = unpackRectangle(values);
		rectangles.push_back(rect);
	}

	ControlValue controlValue;
	controlValue.set(Span<const Rectangle>(rectangles));

	return controlValue;
}

std::vector<std::vector<std::string>> CaptureScript::parseArrays()
{
	EventPtr event = nextEvent(YAML_SEQUENCE_START_EVENT);
	if (!event)
		return {};

	event = nextEvent();
	if (!event)
		return {};

	std::vector<std::vector<std::string>> valueArrays;

	/* Parse single array. */
	if (event->type == YAML_SCALAR_EVENT) {
		std::string firstValue = eventScalarValue(event);
		if (firstValue.empty())
			return {};

		std::vector<std::string> remaining = parseSingleArray();

		std::vector<std::string> values = { firstValue };
		values.insert(std::end(values),
			      std::begin(remaining), std::end(remaining));
		valueArrays.push_back(values);

		return valueArrays;
	}

	/* Parse array of arrays. */
	while (1) {
		switch (event->type) {
		case YAML_SEQUENCE_START_EVENT: {
			std::vector<std::string> values = parseSingleArray();
			valueArrays.push_back(values);
			break;
		}
		case YAML_SEQUENCE_END_EVENT:
			return valueArrays;
		default:
			return {};
		}

		event = nextEvent();
		if (!event)
			return {};
	}
}

std::vector<std::string> CaptureScript::parseSingleArray()
{
	std::vector<std::string> values;

	while (1) {
		EventPtr event = nextEvent();
		if (!event)
			return {};

		switch (event->type) {
		case YAML_SCALAR_EVENT: {
			std::string value = eventScalarValue(event);
			if (value.empty())
				return {};
			values.push_back(value);
			break;
		}
		case YAML_SEQUENCE_END_EVENT:
			return values;
		default:
			return {};
		}
	}
}

void CaptureScript::unpackFailure(const ControlId *id, const std::string &repr)
{
	static const std::map<unsigned int, const char *> typeNames = {
		{ ControlTypeNone, "none" },
		{ ControlTypeBool, "bool" },
		{ ControlTypeByte, "byte" },
		{ ControlTypeInteger32, "int32" },
		{ ControlTypeInteger64, "int64" },
		{ ControlTypeFloat, "float" },
		{ ControlTypeString, "string" },
		{ ControlTypeRectangle, "Rectangle" },
		{ ControlTypeSize, "Size" },
	};

	const char *typeName;
	auto it = typeNames.find(id->type());
	if (it != typeNames.end())
		typeName = it->second;
	else
		typeName = "unknown";

	std::cerr << "Unsupported control '" << repr << "' for "
		  << typeName << " control " << id->name() << std::endl;
}

ControlValue CaptureScript::parseScalarControl(const ControlId *id,
					       const std::string repr)
{
	ControlValue value{};

	switch (id->type()) {
	case ControlTypeNone:
		break;
	case ControlTypeBool: {
		bool val;

		if (repr == "true") {
			val = true;
		} else if (repr == "false") {
			val = false;
		} else {
			unpackFailure(id, repr);
			return value;
		}

		value.set<bool>(val);
		break;
	}
	case ControlTypeByte: {
		uint8_t val = strtol(repr.c_str(), NULL, 10);
		value.set<uint8_t>(val);
		break;
	}
	case ControlTypeInteger32: {
		int32_t val = strtol(repr.c_str(), NULL, 10);
		value.set<int32_t>(val);
		break;
	}
	case ControlTypeInteger64: {
		int64_t val = strtoll(repr.c_str(), NULL, 10);
		value.set<int64_t>(val);
		break;
	}
	case ControlTypeFloat: {
		float val = strtof(repr.c_str(), NULL);
		value.set<float>(val);
		break;
	}
	case ControlTypeString: {
		value.set<std::string>(repr);
		break;
	}
	default:
		std::cerr << "Unsupported control type" << std::endl;
		break;
	}

	return value;
}

ControlValue CaptureScript::parseArrayControl(const ControlId *id,
					      const std::vector<std::string> &repr)
{
	ControlValue value{};

	switch (id->type()) {
	case ControlTypeNone:
		break;
	case ControlTypeBool: {
		/*
		 * This is unpleasant, but we cannot use an std::vector<> as its
		 * boolean type overload does not allow to access the raw data,
		 * as boolean values are stored in a bitmask for efficiency.
		 *
		 * As we need a contiguous memory region to wrap in a Span<>,
		 * use an array instead but be strict about not overflowing it
		 * by limiting the number of controls we can store.
		 *
		 * Be loud but do not fail, as the issue would present at
		 * runtime and it's not fatal.
		 */
		static constexpr unsigned int kMaxNumBooleanControls = 1024;
		std::array<bool, kMaxNumBooleanControls> values;
		unsigned int idx = 0;

		for (const std::string &s : repr) {
			bool val;

			if (s == "true") {
				val = true;
			} else if (s == "false") {
				val = false;
			} else {
				unpackFailure(id, s);
				return value;
			}

			if (idx == kMaxNumBooleanControls) {
				std::cerr << "Cannot parse more than "
					  << kMaxNumBooleanControls
					  << " boolean controls" << std::endl;
				break;
			}

			values[idx++] = val;
		}

		value = Span<bool>(values.data(), idx);
		break;
	}
	case ControlTypeByte: {
		std::vector<uint8_t> values;
		for (const std::string &s : repr) {
			uint8_t val = strtoll(s.c_str(), NULL, 10);
			values.push_back(val);
		}

		value = Span<const uint8_t>(values.data(), values.size());
		break;
	}
	case ControlTypeInteger32: {
		std::vector<int32_t> values;
		for (const std::string &s : repr) {
			int32_t val = strtoll(s.c_str(), NULL, 10);
			values.push_back(val);
		}

		value = Span<const int32_t>(values.data(), values.size());
		break;
	}
	case ControlTypeInteger64: {
		std::vector<int64_t> values;
		for (const std::string &s : repr) {
			int64_t val = strtoll(s.c_str(), NULL, 10);
			values.push_back(val);
		}

		value = Span<const int64_t>(values.data(), values.size());
		break;
	}
	case ControlTypeFloat: {
		std::vector<float> values;
		for (const std::string &s : repr)
			values.push_back(strtof(s.c_str(), NULL));

		value = Span<const float>(values.data(), values.size());
		break;
	}
	case ControlTypeString: {
		value = Span<const std::string>(repr.data(), repr.size());
		break;
	}
	default:
		std::cerr << "Unsupported control type" << std::endl;
		break;
	}

	return value;
}

ControlValue CaptureScript::unpackControl(const ControlId *id)
{
	/* Parse complex types. */
	switch (id->type()) {
	case ControlTypeRectangle:
		return parseRectangles();
	case ControlTypeSize:
		/* \todo Parse Sizes. */
		return {};
	default:
		break;
	}

	/* Check if the control has a single scalar value or is an array. */
	EventPtr event = nextEvent();
	if (!event)
		return {};

	switch (event->type) {
	case YAML_SCALAR_EVENT: {
		const std::string repr = eventScalarValue(event);
		if (repr.empty())
			return {};

		return parseScalarControl(id, repr);
	}
	case YAML_SEQUENCE_START_EVENT: {
		std::vector<std::string> array = parseSingleArray();
		if (array.empty())
			return {};

		return parseArrayControl(id, array);
	}
	default:
		std::cerr << "Unexpected event type: " << event->type << std::endl;
		return {};
	}
}

libcamera::Rectangle CaptureScript::unpackRectangle(const std::vector<std::string> &strVec)
{
	int x = strtol(strVec[0].c_str(), NULL, 10);
	int y = strtol(strVec[1].c_str(), NULL, 10);
	unsigned int width = strtoul(strVec[2].c_str(), NULL, 10);
	unsigned int height = strtoul(strVec[3].c_str(), NULL, 10);

	return Rectangle(x, y, width, height);
}
