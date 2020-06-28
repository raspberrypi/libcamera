/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_context_wrapper.cpp - Image Processing Algorithm context wrapper
 */

#include "libcamera/internal/ipa_context_wrapper.h"

#include <vector>

#include <libcamera/controls.h>

#include "libcamera/internal/byte_stream_buffer.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/utils.h"

/**
 * \file ipa_context_wrapper.h
 * \brief Image Processing Algorithm context wrapper
 */

namespace libcamera {

/**
 * \class IPAContextWrapper
 * \brief Wrap an ipa_context and expose it as an IPAInterface
 *
 * The IPAContextWrapper class wraps an ipa_context, provided by an IPA module, and
 * exposes an IPAInterface. This mechanism is used for IPAs that are not
 * isolated in a separate process to allow direct calls from pipeline handler
 * using the IPAInterface API instead of the lower-level ipa_context API.
 *
 * The IPAInterface methods are converted to the ipa_context API by translating
 * all C++ arguments into plain C structures or byte arrays that contain no
 * pointer, as required by the ipa_context API.
 */

/**
 * \brief Construct an IPAContextWrapper instance that wraps the \a context
 * \param[in] context The IPA module context
 *
 * Ownership of the \a context is passed to the IPAContextWrapper. The context remains
 * valid for the whole lifetime of the wrapper and is destroyed automatically
 * with it.
 */
IPAContextWrapper::IPAContextWrapper(struct ipa_context *context)
	: ctx_(context), intf_(nullptr)
{
	if (!ctx_)
		return;

	bool forceCApi = !!utils::secure_getenv("LIBCAMERA_IPA_FORCE_C_API");

	if (!forceCApi && ctx_ && ctx_->ops->get_interface) {
		intf_ = reinterpret_cast<IPAInterface *>(ctx_->ops->get_interface(ctx_));
		intf_->queueFrameAction.connect(this, &IPAContextWrapper::doQueueFrameAction);
		return;
	}

	ctx_->ops->register_callbacks(ctx_, &IPAContextWrapper::callbacks_,
				      this);
}

IPAContextWrapper::~IPAContextWrapper()
{
	if (!ctx_)
		return;

	ctx_->ops->destroy(ctx_);
}

int IPAContextWrapper::init(const IPASettings &settings)
{
	if (intf_)
		return intf_->init(settings);

	if (!ctx_)
		return 0;

	struct ipa_settings c_settings;
	c_settings.configuration_file = settings.configurationFile.c_str();

	ctx_->ops->init(ctx_, &c_settings);

	return 0;
}

int IPAContextWrapper::start()
{
	if (intf_)
		return intf_->start();

	if (!ctx_)
		return 0;

	return ctx_->ops->start(ctx_);
}

void IPAContextWrapper::stop()
{
	if (intf_)
		return intf_->stop();

	if (!ctx_)
		return;

	ctx_->ops->stop(ctx_);
}

void IPAContextWrapper::configure(const CameraSensorInfo &sensorInfo,
				  const std::map<unsigned int, IPAStream> &streamConfig,
				  const std::map<unsigned int, const ControlInfoMap &> &entityControls,
				  const IPAOperationData &ipaConfig,
				  IPAOperationData *result)
{
	if (intf_)
		return intf_->configure(sensorInfo, streamConfig,
					entityControls, ipaConfig, result);

	if (!ctx_)
		return;

	serializer_.reset();

	/* Translate the camera sensor info. */
	struct ipa_sensor_info sensor_info = {};
	sensor_info.model = sensorInfo.model.c_str();
	sensor_info.bits_per_pixel = sensorInfo.bitsPerPixel;
	sensor_info.active_area.width = sensorInfo.activeAreaSize.width;
	sensor_info.active_area.height = sensorInfo.activeAreaSize.height;
	sensor_info.analog_crop.left = sensorInfo.analogCrop.x;
	sensor_info.analog_crop.top = sensorInfo.analogCrop.y;
	sensor_info.analog_crop.width = sensorInfo.analogCrop.width;
	sensor_info.analog_crop.height = sensorInfo.analogCrop.height;
	sensor_info.output_size.width = sensorInfo.outputSize.width;
	sensor_info.output_size.height = sensorInfo.outputSize.height;
	sensor_info.pixel_rate = sensorInfo.pixelRate;
	sensor_info.line_length = sensorInfo.lineLength;

	/* Translate the IPA stream configurations map. */
	struct ipa_stream c_streams[streamConfig.size()];

	unsigned int i = 0;
	for (const auto &stream : streamConfig) {
		struct ipa_stream *c_stream = &c_streams[i];
		unsigned int id = stream.first;
		const IPAStream &ipaStream = stream.second;

		c_stream->id = id;
		c_stream->pixel_format = ipaStream.pixelFormat;
		c_stream->width = ipaStream.size.width;
		c_stream->height = ipaStream.size.height;

		++i;
	}

	/* Translate the IPA entity controls map. */
	struct ipa_control_info_map c_info_maps[entityControls.size()];
	std::vector<std::vector<uint8_t>> data(entityControls.size());

	i = 0;
	for (const auto &info : entityControls) {
		struct ipa_control_info_map &c_info_map = c_info_maps[i];
		unsigned int id = info.first;
		const ControlInfoMap &infoMap = info.second;

		size_t infoMapSize = serializer_.binarySize(infoMap);
		data[i].resize(infoMapSize);
		ByteStreamBuffer byteStream(data[i].data(), data[i].size());
		serializer_.serialize(infoMap, byteStream);

		c_info_map.id = id;
		c_info_map.data = byteStream.base();
		c_info_map.size = byteStream.size();

		++i;
	}

	/* \todo Translate the ipaConfig and reponse */
	ctx_->ops->configure(ctx_, &sensor_info, c_streams, streamConfig.size(),
			     c_info_maps, entityControls.size());
}

void IPAContextWrapper::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	if (intf_)
		return intf_->mapBuffers(buffers);

	if (!ctx_)
		return;

	struct ipa_buffer c_buffers[buffers.size()];

	for (unsigned int i = 0; i < buffers.size(); ++i) {
		struct ipa_buffer &c_buffer = c_buffers[i];
		const IPABuffer &buffer = buffers[i];
		const std::vector<FrameBuffer::Plane> &planes = buffer.planes;

		c_buffer.id = buffer.id;
		c_buffer.num_planes = planes.size();

		for (unsigned int j = 0; j < planes.size(); ++j) {
			const FrameBuffer::Plane &plane = planes[j];
			c_buffer.planes[j].dmabuf = plane.fd.fd();
			c_buffer.planes[j].length = plane.length;
		}
	}

	ctx_->ops->map_buffers(ctx_, c_buffers, buffers.size());
}

void IPAContextWrapper::unmapBuffers(const std::vector<unsigned int> &ids)
{
	if (intf_)
		return intf_->unmapBuffers(ids);

	if (!ctx_)
		return;

	ctx_->ops->unmap_buffers(ctx_, ids.data(), ids.size());
}

void IPAContextWrapper::processEvent(const IPAOperationData &data)
{
	if (intf_)
		return intf_->processEvent(data);

	if (!ctx_)
		return;

	struct ipa_operation_data c_data;
	c_data.operation = data.operation;
	c_data.data = data.data.data();
	c_data.num_data = data.data.size();

	struct ipa_control_list control_lists[data.controls.size()];
	c_data.lists = control_lists;
	c_data.num_lists = data.controls.size();

	std::size_t listsSize = 0;
	for (const auto &list : data.controls)
		listsSize += serializer_.binarySize(list);

	std::vector<uint8_t> binaryData(listsSize);
	ByteStreamBuffer byteStreamBuffer(binaryData.data(), listsSize);

	unsigned int i = 0;
	for (const auto &list : data.controls) {
		struct ipa_control_list &c_list = control_lists[i];
		c_list.size = serializer_.binarySize(list);
		ByteStreamBuffer b = byteStreamBuffer.carveOut(c_list.size);

		serializer_.serialize(list, b);

		c_list.data = b.base();
	}

	ctx_->ops->process_event(ctx_, &c_data);
}

void IPAContextWrapper::doQueueFrameAction(unsigned int frame,
					   const IPAOperationData &data)
{
	IPAInterface::queueFrameAction.emit(frame, data);
}

void IPAContextWrapper::queue_frame_action(void *ctx, unsigned int frame,
					   struct ipa_operation_data &data)
{
	IPAContextWrapper *_this = static_cast<IPAContextWrapper *>(ctx);
	IPAOperationData opData;

	opData.operation = data.operation;
	for (unsigned int i = 0; i < data.num_data; ++i)
		opData.data.push_back(data.data[i]);

	for (unsigned int i = 0; i < data.num_lists; ++i) {
		const struct ipa_control_list &c_list = data.lists[i];
		ByteStreamBuffer b(c_list.data, c_list.size);
		opData.controls.push_back(_this->serializer_.deserialize<ControlList>(b));
	}

	_this->doQueueFrameAction(frame, opData);
}

#ifndef __DOXYGEN__
/*
 * This construct confuses Doxygen and makes it believe that all members of the
 * operations is a member of IPAContextWrapper. It must thus be hidden.
 */
const struct ipa_callback_ops IPAContextWrapper::callbacks_ = {
	.queue_frame_action = &IPAContextWrapper::queue_frame_action,
};
#endif

} /* namespace libcamera */
