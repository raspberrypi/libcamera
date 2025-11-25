/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * DW100 Dewarp Engine integration
 */

#pragma once

#include <memory>
#include <queue>

#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>

#include "libcamera/internal/converter/converter_dw100_vertexmap.h"
#include "libcamera/internal/converter/converter_v4l2_m2m.h"
#include "libcamera/internal/device_enumerator.h"

namespace libcamera {

class MediaDevice;
class Rectangle;
class Stream;

class ConverterDW100Module
{
public:
	virtual ~ConverterDW100Module() = default;

	static std::unique_ptr<ConverterDW100Module> createModule(DeviceEnumerator *enumerator);

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>>
			      &outputCfg);
	bool isConfigured(const Stream *stream) const;

	Size adjustInputSize(const PixelFormat &pixFmt, const Size &size,
			     Converter::Alignment align = Converter::Alignment::Down);
	Size adjustOutputSize(const PixelFormat &pixFmt, const Size &size,
			      Converter::Alignment align = Converter::Alignment::Down);

	int exportBuffers(const Stream *stream, unsigned int count,
			  std::vector<std::unique_ptr<FrameBuffer>> *buffers);
	int validateOutput(StreamConfiguration *cfg, bool *adjusted,
			   Converter::Alignment align = Converter::Alignment::Down);
	int queueBuffers(FrameBuffer *input,
			 const std::map<const Stream *, FrameBuffer *> &outputs);

	int start();
	void stop();

	void updateControlInfos(const Stream *stream, ControlInfoMap::Map &infos);
	void setControls(const Stream *stream, const ControlList &controls);
	void populateMetadata(const Stream *stream, ControlList &meta);

	void setSensorCrop(const Rectangle &rect);
	void setTransform(const Stream *stream, const Transform &transform);

	Signal<FrameBuffer *> inputBufferReady;
	Signal<FrameBuffer *> outputBufferReady;

private:
	ConverterDW100Module(std::shared_ptr<MediaDevice> media);

	int applyControls(const Stream *stream, const V4L2Request *request);
	void reinitRequest(V4L2Request *request);

	struct VertexMapInfo {
		Dw100VertexMap map;
		bool update;
	};

	std::map<const Stream *, VertexMapInfo> vertexMaps_;
	unsigned int inputBufferCount_;
	V4L2M2MConverter converter_;
	Rectangle sensorCrop_;
	bool running_;

	std::vector<std::unique_ptr<V4L2Request>> requests_;
	std::queue<V4L2Request *> availableRequests_;
};

} /* namespace libcamera */
