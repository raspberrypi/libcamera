/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * imgu.h - Intel IPU3 ImgU
 */
#ifndef __LIBCAMERA_PIPELINE_IPU3_IMGU_H__
#define __LIBCAMERA_PIPELINE_IPU3_IMGU_H__

#include <memory>
#include <string>

#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class FrameBuffer;
class MediaDevice;
class Size;
struct StreamConfiguration;

class ImgUDevice
{
public:
	struct PipeConfig {
		float bds_sf;
		Size iif;
		Size bds;
		Size gdc;

		bool isNull() const
		{
			return iif.isNull() || bds.isNull() || gdc.isNull();
		}
	};

	struct Pipe {
		Size input;
		Size main;
		Size viewfinder;
	};

	int init(MediaDevice *media, unsigned int index);

	PipeConfig calculatePipeConfig(Pipe *pipe);

	int configure(const PipeConfig &pipeConfig, V4L2DeviceFormat *inputFormat);

	int configureOutput(const StreamConfiguration &cfg,
			    V4L2DeviceFormat *outputFormat)
	{
		return configureVideoDevice(output_.get(), PAD_OUTPUT, cfg,
					    outputFormat);
	}

	int configureViewfinder(const StreamConfiguration &cfg,
				V4L2DeviceFormat *outputFormat)
	{
		return configureVideoDevice(viewfinder_.get(), PAD_VF, cfg,
					    outputFormat);
	}

	int configureStat(const StreamConfiguration &cfg,
			  V4L2DeviceFormat *outputFormat)
	{
		return configureVideoDevice(stat_.get(), PAD_STAT, cfg,
					    outputFormat);
	}

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int start();
	int stop();

	int enableLinks(bool enable);

	std::unique_ptr<V4L2Subdevice> imgu_;
	std::unique_ptr<V4L2VideoDevice> input_;
	std::unique_ptr<V4L2VideoDevice> output_;
	std::unique_ptr<V4L2VideoDevice> viewfinder_;
	std::unique_ptr<V4L2VideoDevice> stat_;
	/* \todo Add param video device for 3A tuning */

private:
	static constexpr unsigned int PAD_INPUT = 0;
	static constexpr unsigned int PAD_OUTPUT = 2;
	static constexpr unsigned int PAD_VF = 3;
	static constexpr unsigned int PAD_STAT = 4;

	int linkSetup(const std::string &source, unsigned int sourcePad,
		      const std::string &sink, unsigned int sinkPad,
		      bool enable);

	int configureVideoDevice(V4L2VideoDevice *dev, unsigned int pad,
				 const StreamConfiguration &cfg,
				 V4L2DeviceFormat *outputFormat);

	std::string name_;
	MediaDevice *media_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_PIPELINE_IPU3_IMGU_H__ */
