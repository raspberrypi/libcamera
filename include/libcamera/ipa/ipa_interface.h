/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_interface.h - Image Processing Algorithm interface
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_H__
#define __LIBCAMERA_IPA_INTERFACE_H__

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ipa_context {
	const struct ipa_context_ops *ops;
};

struct ipa_settings {
	const char *configuration_file;
};

struct ipa_sensor_info {
	const char *model;
	uint8_t bits_per_pixel;
	struct {
		uint32_t width;
		uint32_t height;
	} active_area;
	struct {
		int32_t left;
		int32_t top;
		uint32_t width;
		uint32_t height;
	} analog_crop;
	struct {
		uint32_t width;
		uint32_t height;
	} output_size;
	uint64_t pixel_rate;
	uint32_t line_length;
};

struct ipa_stream {
	unsigned int id;
	unsigned int pixel_format;
	unsigned int width;
	unsigned int height;
};

struct ipa_control_info_map {
	unsigned int id;
	const uint8_t *data;
	size_t size;
};

struct ipa_buffer_plane {
	int dmabuf;
	size_t length;
};

struct ipa_buffer {
	unsigned int id;
	unsigned int num_planes;
	struct ipa_buffer_plane planes[3];
};

struct ipa_control_list {
	const uint8_t *data;
	unsigned int size;
};

struct ipa_operation_data {
	unsigned int operation;
	const uint32_t *data;
	unsigned int num_data;
	const struct ipa_control_list *lists;
	unsigned int num_lists;
};

struct ipa_callback_ops {
	void (*queue_frame_action)(void *cb_ctx, unsigned int frame,
				   struct ipa_operation_data &data);
};

struct ipa_context_ops {
	void (*destroy)(struct ipa_context *ctx);
	void *(*get_interface)(struct ipa_context *ctx);
	void (*init)(struct ipa_context *ctx,
		     const struct ipa_settings *settings);
	int (*start)(struct ipa_context *ctx);
	void (*stop)(struct ipa_context *ctx);
	void (*register_callbacks)(struct ipa_context *ctx,
				   const struct ipa_callback_ops *callbacks,
				   void *cb_ctx);
	void (*configure)(struct ipa_context *ctx,
			  const struct ipa_sensor_info *sensor_info,
			  const struct ipa_stream *streams,
			  unsigned int num_streams,
			  const struct ipa_control_info_map *maps,
			  unsigned int num_maps);
	void (*map_buffers)(struct ipa_context *ctx,
			    const struct ipa_buffer *buffers,
			    size_t num_buffers);
	void (*unmap_buffers)(struct ipa_context *ctx, const unsigned int *ids,
			      size_t num_buffers);
	void (*process_event)(struct ipa_context *ctx,
			      const struct ipa_operation_data *data);
};

struct ipa_context *ipaCreate();

#ifdef __cplusplus
}

#include <map>
#include <vector>

#include <libcamera/buffer.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/signal.h>

namespace libcamera {

struct IPASettings {
	std::string configurationFile;
};

struct IPAStream {
	unsigned int pixelFormat;
	Size size;
};

struct IPABuffer {
	unsigned int id;
	std::vector<FrameBuffer::Plane> planes;
};

struct IPAOperationData {
	unsigned int operation;
	std::vector<uint32_t> data;
	std::vector<ControlList> controls;
};

struct CameraSensorInfo;

class IPAInterface
{
public:
	virtual ~IPAInterface() {}

	virtual int init(const IPASettings &settings) = 0;
	virtual int start() = 0;
	virtual void stop() = 0;

	virtual void configure(const CameraSensorInfo &sensorInfo,
			       const std::map<unsigned int, IPAStream> &streamConfig,
			       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
			       const IPAOperationData &ipaConfig,
			       IPAOperationData *result) = 0;

	virtual void mapBuffers(const std::vector<IPABuffer> &buffers) = 0;
	virtual void unmapBuffers(const std::vector<unsigned int> &ids) = 0;

	virtual void processEvent(const IPAOperationData &data) = 0;
	Signal<unsigned int, const IPAOperationData &> queueFrameAction;
};

} /* namespace libcamera */
#endif

#endif /* __LIBCAMERA_IPA_INTERFACE_H__ */
