/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * imgu.cpp - Intel IPU3 ImgU
 */

#include "imgu.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <linux/media-bus-format.h>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/media_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPU3)

namespace {

/*
 * The procedure to calculate the ImgU pipe configuration has been ported
 * from the pipe_config.py python script, available at:
 * https://github.com/intel/intel-ipu3-pipecfg
 * at revision: 243d13446e44 ("Fix some bug for some resolutions")
 */

/* BSD scaling factors: min=1, max=2.5, step=1/32 */
const std::vector<float> bdsScalingFactors = {
	1, 1.03125, 1.0625, 1.09375, 1.125, 1.15625, 1.1875, 1.21875, 1.25,
	1.28125, 1.3125, 1.34375, 1.375, 1.40625, 1.4375, 1.46875, 1.5, 1.53125,
	1.5625, 1.59375, 1.625, 1.65625, 1.6875, 1.71875, 1.75, 1.78125, 1.8125,
	1.84375, 1.875, 1.90625, 1.9375, 1.96875, 2, 2.03125, 2.0625, 2.09375,
	2.125, 2.15625, 2.1875, 2.21875, 2.25, 2.28125, 2.3125, 2.34375, 2.375,
	2.40625, 2.4375, 2.46875, 2.5
};

/* GDC scaling factors: min=1, max=16, step=1/4 */
const std::vector<float> gdcScalingFactors = {
	1, 1.25, 1.5, 1.75, 2, 2.25, 2.5, 2.75, 3, 3.25, 3.5, 3.75, 4, 4.25,
	4.5, 4.75, 5, 5.25, 5.5, 5.75, 6, 6.25, 6.5, 6.75, 7, 7.25, 7.5, 7.75,
	8, 8.25, 8.5, 8.75, 9, 9.25, 9.5, 9.75, 10, 10.25, 10.5, 10.75, 11,
	11.25, 11.5, 11.75, 12, 12.25, 12.5, 12.75, 13, 13.25, 13.5, 13.75, 14,
	14.25, 14.5, 14.75, 15, 15.25, 15.5, 15.75, 16,
};

std::vector<ImgUDevice::PipeConfig> pipeConfigs;

struct FOV {
	float w;
	float h;

	bool isLarger(const FOV &other)
	{
		if (w > other.w)
			return true;
		if (w == other.w && h > other.h)
			return true;
		return false;
	}
};

/* Approximate a scaling factor sf to the closest one available in a range. */
float findScaleFactor(float sf, const std::vector<float> &range,
		      bool roundDown = false)
{
	if (sf <= range[0])
		return range[0];
	if (sf >= range[range.size() - 1])
		return range[range.size() - 1];

	float bestDiff = std::numeric_limits<float>::max();
	unsigned int index = 0;
	for (unsigned int i = 0; i < range.size(); ++i) {
		float diff = utils::abs_diff(sf, range[i]);
		if (diff < bestDiff) {
			bestDiff = diff;
			index = i;
		}
	}

	if (roundDown && index > 0 && sf < range[index])
		index--;

	return range[index];
}

bool isSameRatio(const Size &in, const Size &out)
{
	float inRatio = static_cast<float>(in.width) / in.height;
	float outRatio = static_cast<float>(out.width) / out.height;

	if (utils::abs_diff(inRatio, outRatio) > 0.1)
		return false;

	return true;
}

void calculateBDSHeight(ImgUDevice::Pipe *pipe, const Size &iif, const Size &gdc,
			unsigned int bdsWidth, float bdsSF)
{
	unsigned int minIFHeight = iif.height - ImgUDevice::kIFMaxCropHeight;
	unsigned int minBDSHeight = gdc.height + ImgUDevice::kFilterHeight * 2;
	unsigned int ifHeight;
	float bdsHeight;

	if (!isSameRatio(pipe->input, gdc)) {
		unsigned int foundIfHeight = 0;
		float estIFHeight = (iif.width * gdc.height) /
				    static_cast<float>(gdc.width);
		estIFHeight = std::clamp<float>(estIFHeight, minIFHeight, iif.height);

		ifHeight = utils::alignUp(estIFHeight, ImgUDevice::kIFAlignHeight);
		while (ifHeight >= minIFHeight && ifHeight <= iif.height &&
		       ifHeight / bdsSF >= minBDSHeight) {

			float height = ifHeight / bdsSF;
			if (std::fmod(height, 1.0) == 0) {
				unsigned int bdsIntHeight = static_cast<unsigned int>(height);

				if (!(bdsIntHeight % ImgUDevice::kBDSAlignHeight)) {
					foundIfHeight = ifHeight;
					bdsHeight = height;
					break;
				}
			}

			ifHeight -= ImgUDevice::kIFAlignHeight;
		}

		ifHeight = utils::alignUp(estIFHeight, ImgUDevice::kIFAlignHeight);
		while (ifHeight >= minIFHeight && ifHeight <= iif.height &&
		       ifHeight / bdsSF >= minBDSHeight) {

			float height = ifHeight / bdsSF;
			if (std::fmod(height, 1.0) == 0) {
				unsigned int bdsIntHeight = static_cast<unsigned int>(height);

				if (!(bdsIntHeight % ImgUDevice::kBDSAlignHeight)) {
					foundIfHeight = ifHeight;
					bdsHeight = height;
					break;
				}
			}

			ifHeight += ImgUDevice::kIFAlignHeight;
		}

		if (foundIfHeight) {
			unsigned int bdsIntHeight = static_cast<unsigned int>(bdsHeight);

			pipeConfigs.push_back({ bdsSF, { iif.width, foundIfHeight },
						{ bdsWidth, bdsIntHeight }, gdc });
			return;
		}
	} else {
		ifHeight = utils::alignUp(iif.height, ImgUDevice::kIFAlignHeight);
		while (ifHeight >= minIFHeight && ifHeight / bdsSF >= minBDSHeight) {

			bdsHeight = ifHeight / bdsSF;
			if (std::fmod(ifHeight, 1.0) == 0 && std::fmod(bdsHeight, 1.0) == 0) {
				unsigned int bdsIntHeight = static_cast<unsigned int>(bdsHeight);

				if (!(ifHeight % ImgUDevice::kIFAlignHeight) &&
				    !(bdsIntHeight % ImgUDevice::kBDSAlignHeight)) {
					pipeConfigs.push_back({ bdsSF, { iif.width, ifHeight },
								{ bdsWidth, bdsIntHeight }, gdc });
				}
			}

			ifHeight -= ImgUDevice::kIFAlignHeight;
		}
	}
}

void calculateBDS(ImgUDevice::Pipe *pipe, const Size &iif, const Size &gdc, float bdsSF)
{
	unsigned int minBDSWidth = gdc.width + ImgUDevice::kFilterWidth * 2;
	unsigned int minBDSHeight = gdc.height + ImgUDevice::kFilterHeight * 2;

	float sf = bdsSF;
	while (sf <= ImgUDevice::kBDSSfMax && sf >= ImgUDevice::kBDSSfMin) {
		float bdsWidth = static_cast<float>(iif.width) / sf;
		float bdsHeight = static_cast<float>(iif.height) / sf;

		if (std::fmod(bdsWidth, 1.0) == 0 &&
		    std::fmod(bdsHeight, 1.0) == 0) {
			unsigned int bdsIntWidth = static_cast<unsigned int>(bdsWidth);
			unsigned int bdsIntHeight = static_cast<unsigned int>(bdsHeight);
			if (!(bdsIntWidth % ImgUDevice::kBDSAlignWidth) && bdsWidth >= minBDSWidth &&
			    !(bdsIntHeight % ImgUDevice::kBDSAlignHeight) && bdsHeight >= minBDSHeight)
				calculateBDSHeight(pipe, iif, gdc, bdsIntWidth, sf);
		}

		sf += ImgUDevice::kBDSSfStep;
	}

	sf = bdsSF;
	while (sf <= ImgUDevice::kBDSSfMax && sf >= ImgUDevice::kBDSSfMin) {
		float bdsWidth = static_cast<float>(iif.width) / sf;
		float bdsHeight = static_cast<float>(iif.height) / sf;

		if (std::fmod(bdsWidth, 1.0) == 0 &&
		    std::fmod(bdsHeight, 1.0) == 0) {
			unsigned int bdsIntWidth = static_cast<unsigned int>(bdsWidth);
			unsigned int bdsIntHeight = static_cast<unsigned int>(bdsHeight);
			if (!(bdsIntWidth % ImgUDevice::kBDSAlignWidth) && bdsWidth >= minBDSWidth &&
			    !(bdsIntHeight % ImgUDevice::kBDSAlignHeight) && bdsHeight >= minBDSHeight)
				calculateBDSHeight(pipe, iif, gdc, bdsIntWidth, sf);
		}

		sf -= ImgUDevice::kBDSSfStep;
	}
}

Size calculateGDC(ImgUDevice::Pipe *pipe)
{
	const Size &in = pipe->input;
	const Size &main = pipe->main;
	const Size &vf = pipe->viewfinder;
	Size gdc;

	if (!vf.isNull()) {
		gdc.width = main.width;

		float ratio = (main.width * vf.height) / static_cast<float>(vf.width);
		gdc.height = std::max(static_cast<float>(main.height), ratio);

		return gdc;
	}

	if (!isSameRatio(in, main)) {
		gdc = main;
		return gdc;
	}

	float totalSF = static_cast<float>(in.width) / main.width;
	float bdsSF = totalSF > 2 ? 2 : 1;
	float yuvSF = totalSF / bdsSF;
	float sf = findScaleFactor(yuvSF, gdcScalingFactors);

	gdc.width = main.width * sf;
	gdc.height = main.height * sf;

	return gdc;
}

FOV calcFOV(const Size &in, const ImgUDevice::PipeConfig &pipe)
{
	FOV fov{};

	float inW = static_cast<float>(in.width);
	float inH = static_cast<float>(in.height);
	float ifCropW = static_cast<float>(in.width - pipe.iif.width);
	float ifCropH = static_cast<float>(in.height - pipe.iif.height);
	float gdcCropW = static_cast<float>(pipe.bds.width - pipe.gdc.width) * pipe.bds_sf;
	float gdcCropH = static_cast<float>(pipe.bds.height - pipe.gdc.height) * pipe.bds_sf;

	fov.w = (inW - (ifCropW + gdcCropW)) / inW;
	fov.h = (inH - (ifCropH + gdcCropH)) / inH;

	return fov;
}

} /* namespace */

/**
 * \struct PipeConfig
 * \brief The ImgU pipe configuration parameters
 *
 * The ImgU image pipeline is composed of several hardware blocks that crop
 * and scale the input image to obtain the desired output sizes. The
 * scaling/cropping operations of those components is configured though the
 * V4L2 selection API and the V4L2 subdev API applied to the ImgU media entity.
 *
 * The configurable components in the pipeline are:
 * - IF: image feeder
 * - BDS: bayer downscaler
 * - GDC: geometric distorsion correction
 *
 * The IF crop rectangle is controlled by the V4L2_SEL_TGT_CROP selection target
 * applied to the ImgU media entity sink pad number 0. The BDS scaler is
 * controlled by the V4L2_SEL_TGT_COMPOSE target on the same pad, while the GDC
 * output size is configured with the VIDIOC_SUBDEV_S_FMT IOCTL, again on pad
 * number 0.
 *
 * The PipeConfig structure collects the sizes of each of those components
 * plus the BDS scaling factor used to calculate the field of view
 * of the final images.
 */

/**
 * \struct Pipe
 * \brief Describe the ImgU requested configuration
 *
 * The ImgU unit processes images through several components, which have
 * to be properly configured inspecting the input image size and the desired
 * output sizes. This structure collects the ImgU input configuration and the
 * requested main output and viewfinder configurations.
 *
 * \var Pipe::input
 * \brief The input image size
 *
 * \var Pipe::main
 * \brief The requested main output size
 *
 * \var Pipe::viewfinder
 * \brief The requested viewfinder output size
 */

/**
 * \brief Initialize components of the ImgU instance
 * \param[in] mediaDevice The ImgU instance media device
 * \param[in] index The ImgU instance index
 *
 * Create and open the V4L2 devices and subdevices of the ImgU instance
 * with \a index.
 *
 * In case of errors the created V4L2VideoDevice and V4L2Subdevice instances
 * are destroyed at pipeline handler delete time.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::init(MediaDevice *media, unsigned int index)
{
	int ret;

	name_ = "ipu3-imgu " + std::to_string(index);
	media_ = media;

	/*
	 * The media entities presence in the media device has been verified
	 * by the match() function: no need to check for newly created
	 * video devices and subdevice validity here.
	 */
	imgu_ = V4L2Subdevice::fromEntityName(media, name_);
	ret = imgu_->open();
	if (ret)
		return ret;

	input_ = V4L2VideoDevice::fromEntityName(media, name_ + " input");
	ret = input_->open();
	if (ret)
		return ret;

	output_ = V4L2VideoDevice::fromEntityName(media, name_ + " output");
	ret = output_->open();
	if (ret)
		return ret;

	viewfinder_ = V4L2VideoDevice::fromEntityName(media, name_ + " viewfinder");
	ret = viewfinder_->open();
	if (ret)
		return ret;

	param_ = V4L2VideoDevice::fromEntityName(media, name_ + " parameters");
	ret = param_->open();
	if (ret)
		return ret;

	stat_ = V4L2VideoDevice::fromEntityName(media, name_ + " 3a stat");
	ret = stat_->open();
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Calculate the ImgU pipe configuration parameters
 * \param[in] pipe The requested ImgU configuration
 * \return An ImgUDevice::PipeConfig instance on success, an empty configuration
 * otherwise
 */
ImgUDevice::PipeConfig ImgUDevice::calculatePipeConfig(Pipe *pipe)
{
	pipeConfigs.clear();

	LOG(IPU3, Debug) << "Calculating pipe configuration for: ";
	LOG(IPU3, Debug) << "input: " << pipe->input;
	LOG(IPU3, Debug) << "main: " << pipe->main;
	LOG(IPU3, Debug) << "vf: " << pipe->viewfinder;

	const Size &in = pipe->input;

	/*
	 * \todo Filter out all resolutions < IF_CROP_MAX.
	 * See https://bugs.libcamera.org/show_bug.cgi?id=32
	 */
	if (in.width < ImgUDevice::kIFMaxCropWidth || in.height < ImgUDevice::kIFMaxCropHeight) {
		LOG(IPU3, Error) << "Input resolution " << in << " not supported";
		return {};
	}

	Size gdc = calculateGDC(pipe);

	float bdsSF = static_cast<float>(in.width) / gdc.width;
	float sf = findScaleFactor(bdsSF, bdsScalingFactors, true);

	/* Populate the configurations vector by scaling width and height. */
	unsigned int ifWidth = utils::alignUp(in.width, ImgUDevice::kIFAlignWidth);
	unsigned int ifHeight = utils::alignUp(in.height, ImgUDevice::kIFAlignHeight);
	unsigned int minIfWidth = in.width - ImgUDevice::kIFMaxCropWidth;
	unsigned int minIfHeight = in.height - ImgUDevice::kIFMaxCropHeight;
	while (ifWidth >= minIfWidth) {
		while (ifHeight >= minIfHeight) {
			Size iif{ ifWidth, ifHeight };
			calculateBDS(pipe, iif, gdc, sf);
			ifHeight -= ImgUDevice::kIFAlignHeight;
		}

		ifWidth -= ImgUDevice::kIFAlignWidth;
	}

	/* Repeat search by scaling width first. */
	ifWidth = utils::alignUp(in.width, ImgUDevice::kIFAlignWidth);
	ifHeight = utils::alignUp(in.height, ImgUDevice::kIFAlignHeight);
	minIfWidth = in.width - ImgUDevice::kIFMaxCropWidth;
	minIfHeight = in.height - ImgUDevice::kIFMaxCropHeight;
	while (ifHeight >= minIfHeight) {
		/*
		 * \todo This procedure is probably broken:
		 * https://github.com/intel/intel-ipu3-pipecfg/issues/2
		 */
		while (ifWidth >= minIfWidth) {
			Size iif{ ifWidth, ifHeight };
			calculateBDS(pipe, iif, gdc, sf);
			ifWidth -= ImgUDevice::kIFAlignWidth;
		}

		ifHeight -= ImgUDevice::kIFAlignHeight;
	}

	if (pipeConfigs.size() == 0) {
		LOG(IPU3, Error) << "Failed to calculate pipe configuration";
		return {};
	}

	FOV bestFov = calcFOV(pipe->input, pipeConfigs[0]);
	unsigned int bestIndex = 0;
	unsigned int p = 0;
	for (auto pipeConfig : pipeConfigs) {
		FOV fov = calcFOV(pipe->input, pipeConfig);
		if (fov.isLarger(bestFov)) {
			bestFov = fov;
			bestIndex = p;
		}

		++p;
	}

	LOG(IPU3, Debug) << "Computed pipe configuration: ";
	LOG(IPU3, Debug) << "IF: " << pipeConfigs[bestIndex].iif;
	LOG(IPU3, Debug) << "BDS: " << pipeConfigs[bestIndex].bds;
	LOG(IPU3, Debug) << "GDC: " << pipeConfigs[bestIndex].gdc;

	return pipeConfigs[bestIndex];
}

/**
 * \brief Configure the ImgU pipeline
 * \param[in] config The ImgU pipe configuration parameters
 * \param[in] inputFormat The format to be applied to ImgU input
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configure(const PipeConfig &pipeConfig, V4L2DeviceFormat *inputFormat)
{
	/* Configure the ImgU input video device with the requested sizes. */
	int ret = input_->setFormat(inputFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU input format = " << *inputFormat;

	/*
	 * \todo The IPU3 driver implementation shall be changed to use the
	 * input sizes as 'ImgU Input' subdevice sizes, and use the desired
	 * GDC output sizes to configure the crop/compose rectangles.
	 *
	 * The current IPU3 driver implementation uses GDC sizes as the
	 * 'ImgU Input' subdevice sizes, and the input video device sizes
	 * to configure the crop/compose rectangles, contradicting the
	 * V4L2 specification.
	 */
	Rectangle iif{ 0, 0, pipeConfig.iif };
	ret = imgu_->setSelection(PAD_INPUT, V4L2_SEL_TGT_CROP, &iif);
	if (ret)
		return ret;
	LOG(IPU3, Debug) << "ImgU IF rectangle = " << iif;

	Rectangle bds{ 0, 0, pipeConfig.bds };
	ret = imgu_->setSelection(PAD_INPUT, V4L2_SEL_TGT_COMPOSE, &bds);
	if (ret)
		return ret;
	LOG(IPU3, Debug) << "ImgU BDS rectangle = " << bds;

	V4L2SubdeviceFormat gdcFormat = {};
	gdcFormat.mbus_code = MEDIA_BUS_FMT_FIXED;
	gdcFormat.size = pipeConfig.gdc;

	ret = imgu_->setFormat(PAD_INPUT, &gdcFormat);
	if (ret)
		return ret;

	LOG(IPU3, Debug) << "ImgU GDC format = " << gdcFormat;

	StreamConfiguration paramCfg = {};
	paramCfg.size = inputFormat->size;
	V4L2DeviceFormat paramFormat;
	ret = configureVideoDevice(param_.get(), PAD_PARAM, paramCfg, &paramFormat);
	if (ret)
		return ret;

	StreamConfiguration statCfg = {};
	statCfg.size = inputFormat->size;
	V4L2DeviceFormat statFormat;
	ret = configureVideoDevice(stat_.get(), PAD_STAT, statCfg, &statFormat);
	if (ret)
		return ret;

	return 0;
}

/**
 * \brief Configure a video device on the ImgU
 * \param[in] dev The video device to configure
 * \param[in] pad The pad of the ImgU subdevice
 * \param[in] cfg The requested configuration
 * \param[out] outputFormat The format set on the video device
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::configureVideoDevice(V4L2VideoDevice *dev, unsigned int pad,
				     const StreamConfiguration &cfg,
				     V4L2DeviceFormat *outputFormat)
{
	V4L2SubdeviceFormat imguFormat = {};
	imguFormat.mbus_code = MEDIA_BUS_FMT_FIXED;
	imguFormat.size = cfg.size;

	int ret = imgu_->setFormat(pad, &imguFormat);
	if (ret)
		return ret;

	/*
	 * No need to apply format to the param or stat video devices as the
	 * driver ignores the operation.
	 */
	if (dev == param_.get() || dev == stat_.get())
		return 0;

	*outputFormat = {};
	outputFormat->fourcc = dev->toV4L2PixelFormat(formats::NV12);
	outputFormat->size = cfg.size;
	outputFormat->planesCount = 2;

	ret = dev->setFormat(outputFormat);
	if (ret)
		return ret;

	const char *name = dev == output_.get() ? "output" : "viewfinder";
	LOG(IPU3, Debug) << "ImgU " << name << " format = "
			 << *outputFormat;

	return 0;
}

/**
 * \brief Allocate buffers for all the ImgU video devices
 */
int ImgUDevice::allocateBuffers(unsigned int bufferCount)
{
	/* Share buffers between CIO2 output and ImgU input. */
	int ret = input_->importBuffers(bufferCount);
	if (ret) {
		LOG(IPU3, Error) << "Failed to import ImgU input buffers";
		return ret;
	}

	ret = param_->allocateBuffers(bufferCount, &paramBuffers_);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to allocate ImgU param buffers";
		goto error;
	}

	ret = stat_->allocateBuffers(bufferCount, &statBuffers_);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to allocate ImgU stat buffers";
		goto error;
	}

	/*
	 * Import buffers for all outputs, regardless of whether the
	 * corresponding stream is active or inactive, as the driver needs
	 * buffers to be requested on the V4L2 devices in order to operate.
	 */
	ret = output_->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to import ImgU output buffers";
		goto error;
	}

	ret = viewfinder_->importBuffers(bufferCount);
	if (ret < 0) {
		LOG(IPU3, Error) << "Failed to import ImgU viewfinder buffers";
		goto error;
	}

	return 0;

error:
	freeBuffers();

	return ret;
}

/**
 * \brief Release buffers for all the ImgU video devices
 */
void ImgUDevice::freeBuffers()
{
	int ret;

	paramBuffers_.clear();
	statBuffers_.clear();

	ret = output_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU output buffers";

	ret = param_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU param buffers";

	ret = stat_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU stat buffers";

	ret = viewfinder_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU viewfinder buffers";

	ret = input_->releaseBuffers();
	if (ret)
		LOG(IPU3, Error) << "Failed to release ImgU input buffers";
}

int ImgUDevice::start()
{
	int ret;

	/* Start the ImgU video devices. */
	ret = output_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU output";
		return ret;
	}

	ret = viewfinder_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU viewfinder";
		return ret;
	}

	ret = param_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU param";
		return ret;
	}

	ret = stat_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU stat";
		return ret;
	}

	ret = input_->streamOn();
	if (ret) {
		LOG(IPU3, Error) << "Failed to start ImgU input";
		return ret;
	}

	return 0;
}

int ImgUDevice::stop()
{
	int ret;

	ret = output_->streamOff();
	ret |= viewfinder_->streamOff();
	ret |= param_->streamOff();
	ret |= stat_->streamOff();
	ret |= input_->streamOff();

	return ret;
}

/**
 * \brief Enable or disable a single link on the ImgU instance
 *
 * This function assumes the media device associated with the ImgU instance
 * is open.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::linkSetup(const std::string &source, unsigned int sourcePad,
			  const std::string &sink, unsigned int sinkPad,
			  bool enable)
{
	MediaLink *link = media_->link(source, sourcePad, sink, sinkPad);
	if (!link) {
		LOG(IPU3, Error)
			<< "Failed to get link: '" << source << "':"
			<< sourcePad << " -> '" << sink << "':" << sinkPad;
		return -ENODEV;
	}

	return link->setEnabled(enable);
}

/**
 * \brief Enable or disable all media links in the ImgU instance to prepare
 * for capture operations
 *
 * \todo This function will probably be removed or changed once links will be
 * enabled or disabled selectively.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ImgUDevice::enableLinks(bool enable)
{
	std::string viewfinderName = name_ + " viewfinder";
	std::string paramName = name_ + " parameters";
	std::string outputName = name_ + " output";
	std::string statName = name_ + " 3a stat";
	std::string inputName = name_ + " input";
	int ret;

	ret = linkSetup(inputName, 0, name_, PAD_INPUT, enable);
	if (ret)
		return ret;

	ret = linkSetup(name_, PAD_OUTPUT, outputName, 0, enable);
	if (ret)
		return ret;

	ret = linkSetup(name_, PAD_VF, viewfinderName, 0, enable);
	if (ret)
		return ret;

	ret = linkSetup(paramName, 0, name_, PAD_PARAM, enable);
	if (ret)
		return ret;

	return linkSetup(name_, PAD_STAT, statName, 0, enable);
}

} /* namespace libcamera */
