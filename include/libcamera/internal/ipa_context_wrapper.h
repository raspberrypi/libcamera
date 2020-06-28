/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_context_wrapper.h - Image Processing Algorithm context wrapper
 */
#ifndef __LIBCAMERA_INTERNAL_IPA_CONTEXT_WRAPPER_H__
#define __LIBCAMERA_INTERNAL_IPA_CONTEXT_WRAPPER_H__

#include <libcamera/ipa/ipa_interface.h>

#include "libcamera/internal/control_serializer.h"

namespace libcamera {

class IPAContextWrapper final : public IPAInterface
{
public:
	IPAContextWrapper(struct ipa_context *context);
	~IPAContextWrapper();

	int init(const IPASettings &settings) override;
	int start() override;
	void stop() override;
	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	virtual void processEvent(const IPAOperationData &data) override;

private:
	static void queue_frame_action(void *ctx, unsigned int frame,
				       struct ipa_operation_data &data);
	static const struct ipa_callback_ops callbacks_;

	void doQueueFrameAction(unsigned int frame,
				const IPAOperationData &data);

	struct ipa_context *ctx_;
	IPAInterface *intf_;

	ControlSerializer serializer_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_IPA_CONTEXT_WRAPPER_H__ */
