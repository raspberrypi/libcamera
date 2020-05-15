/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_interface_wrapper.h - Image Processing Algorithm interface wrapper
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_WRAPPER_H__
#define __LIBCAMERA_IPA_INTERFACE_WRAPPER_H__

#include <memory>

#include <libcamera/ipa/ipa_interface.h>

#include "libcamera/internal/control_serializer.h"

namespace libcamera {

class IPAInterfaceWrapper : public ipa_context
{
public:
	IPAInterfaceWrapper(std::unique_ptr<IPAInterface> interface);

private:
	static void destroy(struct ipa_context *ctx);
	static void *get_interface(struct ipa_context *ctx);
	static void init(struct ipa_context *ctx,
			 const struct ipa_settings *settings);
	static int start(struct ipa_context *ctx);
	static void stop(struct ipa_context *ctx);
	static void register_callbacks(struct ipa_context *ctx,
				       const struct ipa_callback_ops *callbacks,
				       void *cb_ctx);
	static void configure(struct ipa_context *ctx,
			      const struct ipa_sensor_info *sensor_info,
			      const struct ipa_stream *streams,
			      unsigned int num_streams,
			      const struct ipa_control_info_map *maps,
			      unsigned int num_maps);
	static void map_buffers(struct ipa_context *ctx,
				const struct ipa_buffer *c_buffers,
				size_t num_buffers);
	static void unmap_buffers(struct ipa_context *ctx,
				  const unsigned int *ids,
				  size_t num_buffers);
	static void process_event(struct ipa_context *ctx,
				  const struct ipa_operation_data *data);

	static const struct ipa_context_ops operations_;

	void queueFrameAction(unsigned int frame, const IPAOperationData &data);

	std::unique_ptr<IPAInterface> ipa_;
	const struct ipa_callback_ops *callbacks_;
	void *cb_ctx_;

	ControlSerializer serializer_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_INTERFACE_WRAPPER_H__ */
