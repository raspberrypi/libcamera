/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2025 Raspberry Pi Ltd
 *
 * backend_device.cpp - PiSP Backend device helper
 */

#include <cstdint>
#include <cstring>

#include "backend_device.hpp"

using namespace libpisp::helpers;

BackendDevice::BackendDevice(const std::string &device)
	: valid_(true)
{
	nodes_ = MediaDevice().OpenV4l2Nodes(device);
	if (nodes_.empty())
		valid_ = false;

	// Allocate a config buffer to persist.
	nodes_.at("pispbe-config").RequestBuffers(1);
	nodes_.at("pispbe-config").StreamOn();
	config_buffer_ = nodes_.at("pispbe-config").AcquireBuffer().value();
}

BackendDevice::~BackendDevice()
{
	nodes_.at("pispbe-config").StreamOff();
}

void BackendDevice::Setup(const pisp_be_tiles_config &config, unsigned int buffer_count, bool use_opaque_format)
{
	nodes_enabled_.clear();

	if ((config.config.global.rgb_enables & PISP_BE_RGB_ENABLE_INPUT) ||
		(config.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_INPUT))
	{
		nodes_.at("pispbe-input").SetFormat(config.config.input_format, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-input").ReturnBuffers();
		nodes_.at("pispbe-input").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-input");
	}

	if (config.config.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT0)
	{
		nodes_.at("pispbe-output0").SetFormat(config.config.output_format[0].image, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-output0").ReturnBuffers();
		nodes_.at("pispbe-output0").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-output0");
	}

	if (config.config.global.rgb_enables & PISP_BE_RGB_ENABLE_OUTPUT1)
	{
		nodes_.at("pispbe-output1").SetFormat(config.config.output_format[1].image, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-output1").ReturnBuffers();
		nodes_.at("pispbe-output1").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-output1");
	}

	if (config.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_INPUT)
	{
		nodes_.at("pispbe-tdn_input").SetFormat(config.config.tdn_input_format, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-tdn_input").ReturnBuffers();
		nodes_.at("pispbe-tdn_input").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-tdn_input");
	}

	if (config.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_TDN_OUTPUT)
	{
		nodes_.at("pispbe-tdn_output").SetFormat(config.config.tdn_output_format, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-tdn_output").ReturnBuffers();
		nodes_.at("pispbe-tdn_output").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-tdn_output");
	}

	if (config.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_INPUT)
	{
		nodes_.at("pispbe-stitch_input").SetFormat(config.config.stitch_input_format, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-stitch_input").ReturnBuffers();
		nodes_.at("pispbe-stitch_input").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-stitch_input");
	}

	if (config.config.global.bayer_enables & PISP_BE_BAYER_ENABLE_STITCH_OUTPUT)
	{
		nodes_.at("pispbe-stitch_output").SetFormat(config.config.stitch_output_format, use_opaque_format);
		// Release old/allocate a single buffer.
		nodes_.at("pispbe-stitch_output").ReturnBuffers();
		nodes_.at("pispbe-stitch_output").RequestBuffers(buffer_count);
		nodes_enabled_.emplace("pispbe-stitch_output");
	}

	std::memcpy(reinterpret_cast<pisp_be_tiles_config *>(config_buffer_.mem[0]), &config, sizeof(config));
}

std::map<std::string, V4l2Device::Buffer> BackendDevice::AcquireBuffers()
{
	std::map<std::string, V4l2Device::Buffer> buffers;

	for (auto const &n : nodes_enabled_)
		buffers[n] = nodes_.at(n).AcquireBuffer().value();

	return buffers;
}

void BackendDevice::ReleaseBuffer(const std::map<std::string, V4l2Device::Buffer> &buffers)
{
	for (auto const &[n, b] : buffers)
		nodes_.at(n).ReleaseBuffer(b);
}

int BackendDevice::Run(const std::map<std::string, V4l2Device::Buffer> &buffers)
{
	int ret = 0;

	for (auto const &n : nodes_enabled_)
	{
		nodes_.at(n).StreamOn();
		if (nodes_.at(n).QueueBuffer(buffers.at(n).buffer.index))
			ret = -1;
	}

	// Triggers the HW job.
	if (nodes_.at("pispbe-config").QueueBuffer(config_buffer_.buffer.index))
		ret = -1;

	for (auto const &n : nodes_enabled_)
	{
		if (nodes_.at(n).DequeueBuffer(1000) < 0)
			ret = -1;
	}

	// Must dequeue the config buffer in case it's used again.
	if (nodes_.at("pispbe-config").DequeueBuffer(1000) < 0)
		ret = -1;

	for (auto const &n : nodes_enabled_)
		nodes_.at(n).StreamOff();

	return ret;
}
