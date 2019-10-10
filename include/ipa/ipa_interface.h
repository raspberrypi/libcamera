/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_interface.h - Image Processing Algorithm interface
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_H__
#define __LIBCAMERA_IPA_INTERFACE_H__

#include <map>
#include <vector>

#include <libcamera/buffer.h>
#include <libcamera/controls.h>
#include <libcamera/geometry.h>
#include <libcamera/signal.h>

#include "v4l2_controls.h"

namespace libcamera {

struct IPAStream {
	unsigned int pixelFormat;
	Size size;
};

struct IPABuffer {
	unsigned int id;
	BufferMemory memory;
};

struct IPAOperationData {
	unsigned int operation;
	std::vector<uint32_t> data;
	std::vector<ControlList> controls;
};

class IPAInterface
{
public:
	virtual ~IPAInterface() {}

	virtual int init() = 0;

	virtual void configure(const std::map<unsigned int, IPAStream> &streamConfig,
			       const std::map<unsigned int, V4L2ControlInfoMap> &entityControls) = 0;

	virtual void mapBuffers(const std::vector<IPABuffer> &buffers) = 0;
	virtual void unmapBuffers(const std::vector<unsigned int> &ids) = 0;

	virtual void processEvent(const IPAOperationData &data) = 0;
	Signal<unsigned int, const IPAOperationData &> queueFrameAction;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_INTERFACE_H__ */
