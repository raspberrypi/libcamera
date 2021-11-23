/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * imgu.h - Intel IPU3 ImgU
 */

#pragma once

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
	static constexpr unsigned int kFilterWidth = 4;
	static constexpr unsigned int kFilterHeight = 4;

	static constexpr unsigned int kIFAlignWidth = 2;
	static constexpr unsigned int kIFAlignHeight = 4;

	static constexpr unsigned int kIFMaxCropWidth = 40;
	static constexpr unsigned int kIFMaxCropHeight = 540;

	static constexpr unsigned int kBDSAlignWidth = 2;
	static constexpr unsigned int kBDSAlignHeight = 4;

	static constexpr float kBDSSfMax = 2.5;
	static constexpr float kBDSSfMin = 1.0;
	static constexpr float kBDSSfStep = 0.03125;

	static constexpr Size kOutputMinSize = { 2, 2 };
	static constexpr Size kOutputMaxSize = { 4480, 34004 };
	static constexpr unsigned int kOutputAlignWidth = 64;
	static constexpr unsigned int kOutputAlignHeight = 4;
	static constexpr unsigned int kOutputMarginWidth = 64;
	static constexpr unsigned int kOutputMarginHeight = 32;

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

	int allocateBuffers(unsigned int bufferCount);
	void freeBuffers();

	int start();
	int stop();

	int enableLinks(bool enable);

	std::unique_ptr<V4L2Subdevice> imgu_;
	std::unique_ptr<V4L2VideoDevice> input_;
	std::unique_ptr<V4L2VideoDevice> param_;
	std::unique_ptr<V4L2VideoDevice> output_;
	std::unique_ptr<V4L2VideoDevice> viewfinder_;
	std::unique_ptr<V4L2VideoDevice> stat_;

	std::vector<std::unique_ptr<FrameBuffer>> paramBuffers_;
	std::vector<std::unique_ptr<FrameBuffer>> statBuffers_;

private:
	static constexpr unsigned int PAD_INPUT = 0;
	static constexpr unsigned int PAD_PARAM = 1;
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
