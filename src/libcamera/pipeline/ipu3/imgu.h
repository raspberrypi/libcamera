/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * imgu.h - Intel IPU3 ImgU
 */
#ifndef __LIBCAMERA_PIPELINE_IPU3_IMGU_H__
#define __LIBCAMERA_PIPELINE_IPU3_IMGU_H__

#include <string>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
struct Size;
struct StreamConfiguration;

class ImgUDevice
{
public:
	/* ImgU output descriptor: group data specific to an ImgU output. */
	struct ImgUOutput {
		V4L2VideoDevice *dev;
		unsigned int pad;
		std::string name;
	};

	ImgUDevice()
		: imgu_(nullptr), input_(nullptr)
	{
		output_.dev = nullptr;
		viewfinder_.dev = nullptr;
		stat_.dev = nullptr;
	}

	~ImgUDevice()
	{
		delete imgu_;
		delete input_;
		delete output_.dev;
		delete viewfinder_.dev;
		delete stat_.dev;
	}

	int init(MediaDevice *media, unsigned int index);
	int configureInput(const Size &size, V4L2DeviceFormat *inputFormat);
	int configureOutput(ImgUOutput *output, const StreamConfiguration &cfg,
			    V4L2DeviceFormat *outputFormat);

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int start();
	int stop();

	int enableLinks(bool enable);

	V4L2Subdevice *imgu_;
	V4L2VideoDevice *input_;
	ImgUOutput output_;
	ImgUOutput viewfinder_;
	ImgUOutput stat_;
	/* \todo Add param video device for 3A tuning */

private:
	static constexpr unsigned int PAD_INPUT = 0;
	static constexpr unsigned int PAD_OUTPUT = 2;
	static constexpr unsigned int PAD_VF = 3;
	static constexpr unsigned int PAD_STAT = 4;

	int linkSetup(const std::string &source, unsigned int sourcePad,
		      const std::string &sink, unsigned int sinkPad,
		      bool enable);

	std::string name_;
	MediaDevice *media_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_IPU3_IMGU_H__ */
