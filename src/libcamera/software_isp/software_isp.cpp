/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Linaro Ltd
 *
 * Simple software ISP implementation
 */

#include "libcamera/internal/software_isp/software_isp.h"

#include <cmath>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/log.h>
#include <libcamera/base/thread.h>

#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/software_isp/debayer_params.h"

#include "debayer_cpu.h"

/**
 * \file software_isp.cpp
 * \brief Simple software ISP implementation
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(SoftwareIsp)

/**
 * \class SoftwareIsp
 * \brief Class for the Software ISP
 */

/**
 * \var SoftwareIsp::inputBufferReady
 * \brief A signal emitted when the input frame buffer completes
 */

/**
 * \var SoftwareIsp::outputBufferReady
 * \brief A signal emitted when the output frame buffer completes
 */

/**
 * \var SoftwareIsp::ispStatsReady
 * \brief A signal emitted when the statistics for IPA are ready
 */

/**
 * \var SoftwareIsp::metadataReady
 * \brief A signal emitted when the metadata for IPA is ready
 */

/**
 * \var SoftwareIsp::setSensorControls
 * \brief A signal emitted when the values to write to the sensor controls are
 * ready
 */

/**
 * \brief Constructs SoftwareIsp object
 * \param[in] pipe The pipeline handler in use
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[out] ipaControls The IPA controls to update
 * handler
 */
SoftwareIsp::SoftwareIsp(PipelineHandler *pipe, const CameraSensor *sensor,
			 ControlInfoMap *ipaControls)
	: dmaHeap_(DmaBufAllocator::DmaBufAllocatorFlag::CmaHeap |
		   DmaBufAllocator::DmaBufAllocatorFlag::SystemHeap |
		   DmaBufAllocator::DmaBufAllocatorFlag::UDmaBuf)
{
	/*
	 * debayerParams_ must be initialized because the initial value is used for
	 * the first two frames, i.e. until stats processing starts providing its
	 * own parameters.
	 *
	 * \todo This should be handled in the same place as the related
	 * operations, in the IPA module.
	 */
	std::array<uint8_t, 256> gammaTable;
	for (unsigned int i = 0; i < 256; i++)
		gammaTable[i] = UINT8_MAX * std::pow(i / 256.0, 0.5);
	for (unsigned int i = 0; i < DebayerParams::kRGBLookupSize; i++) {
		debayerParams_.red[i] = gammaTable[i];
		debayerParams_.green[i] = gammaTable[i];
		debayerParams_.blue[i] = gammaTable[i];
	}

	if (!dmaHeap_.isValid()) {
		LOG(SoftwareIsp, Error) << "Failed to create DmaBufAllocator object";
		return;
	}

	sharedParams_ = SharedMemObject<DebayerParams>("softIsp_params");
	if (!sharedParams_) {
		LOG(SoftwareIsp, Error) << "Failed to create shared memory for parameters";
		return;
	}

	auto stats = std::make_unique<SwStatsCpu>();
	if (!stats->isValid()) {
		LOG(SoftwareIsp, Error) << "Failed to create SwStatsCpu object";
		return;
	}
	stats->statsReady.connect(this, &SoftwareIsp::statsReady);

	debayer_ = std::make_unique<DebayerCpu>(std::move(stats));
	debayer_->inputBufferReady.connect(this, &SoftwareIsp::inputReady);
	debayer_->outputBufferReady.connect(this, &SoftwareIsp::outputReady);

	ipa_ = IPAManager::createIPA<ipa::soft::IPAProxySoft>(pipe, 0, 0);
	if (!ipa_) {
		LOG(SoftwareIsp, Error)
			<< "Creating IPA for software ISP failed";
		debayer_.reset();
		return;
	}

	/*
	 * The API tuning file is made from the sensor name. If the tuning file
	 * isn't found, fall back to the 'uncalibrated' file.
	 */
	std::string ipaTuningFile =
		ipa_->configurationFile(sensor->model() + ".yaml", "uncalibrated.yaml");

	IPACameraSensorInfo sensorInfo{};
	int ret = sensor->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(SoftwareIsp, Error) << "Camera sensor information not available";
		return;
	}

	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 debayer_->getStatsFD(),
			 sharedParams_.fd(),
			 sensorInfo,
			 sensor->controls(),
			 ipaControls,
			 &ccmEnabled_);
	if (ret) {
		LOG(SoftwareIsp, Error) << "IPA init failed";
		debayer_.reset();
		return;
	}

	ipa_->setIspParams.connect(this, &SoftwareIsp::saveIspParams);
	ipa_->metadataReady.connect(this,
				    [this](uint32_t frame, const ControlList &metadata) {
					    metadataReady.emit(frame, metadata);
				    });
	ipa_->setSensorControls.connect(this, &SoftwareIsp::setSensorCtrls);

	debayer_->moveToThread(&ispWorkerThread_);
}

SoftwareIsp::~SoftwareIsp()
{
	/* make sure to destroy the DebayerCpu before the ispWorkerThread_ is gone */
	debayer_.reset();
}

/**
 * \fn int SoftwareIsp::loadConfiguration([[maybe_unused]] const std::string &filename)
 * \brief Load a configuration from a file
 * \param[in] filename The file to load the configuration data from
 *
 * Currently is a stub doing nothing and always returning "success".
 *
 * \return 0 on success
 */

/**
 * \brief Process the statistics gathered
 * \param[in] frame The frame number
 * \param[in] bufferId ID of the statistics buffer
 * \param[in] sensorControls The sensor controls
 *
 * Requests the IPA to calculate new parameters for ISP and new control
 * values for the sensor.
 */
void SoftwareIsp::processStats(const uint32_t frame, const uint32_t bufferId,
			       const ControlList &sensorControls)
{
	ASSERT(ipa_);
	ipa_->processStats(frame, bufferId, sensorControls);
}

/**
 * \brief Check the validity of Software Isp object
 * \return True if Software Isp is valid, false otherwise
 */
bool SoftwareIsp::isValid() const
{
	return !!debayer_;
}

/**
  * \brief Get the output formats supported for the given input format
  * \param[in] inputFormat The input format
  * \return All the supported output formats or an empty vector if there are none
  */
std::vector<PixelFormat> SoftwareIsp::formats(PixelFormat inputFormat)
{
	ASSERT(debayer_);

	return debayer_->formats(inputFormat);
}

/**
 * \brief Get the supported output sizes for the given input format and size
 * \param[in] inputFormat The input format
 * \param[in] inputSize The input frame size
 * \return The valid size range or an empty range if there are none
 */
SizeRange SoftwareIsp::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	ASSERT(debayer_);

	return debayer_->sizes(inputFormat, inputSize);
}

/**
 * Get the output stride and the frame size in bytes for the given output format and size
 * \param[in] outputFormat The output format
 * \param[in] size The output size (width and height in pixels)
 * \return A tuple of the stride and the frame size in bytes, or a tuple of 0,0
 * if there is no valid output config
 */
std::tuple<unsigned int, unsigned int>
SoftwareIsp::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	ASSERT(debayer_);

	return debayer_->strideAndFrameSize(outputFormat, size);
}

/**
 * \brief Configure the SoftwareIsp object according to the passed in parameters
 * \param[in] inputCfg The input configuration
 * \param[in] outputCfgs The output configurations
 * \param[in] configInfo The IPA configuration data, received from the pipeline
 * handler
 * \return 0 on success, a negative errno on failure
 */
int SoftwareIsp::configure(const StreamConfiguration &inputCfg,
			   const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
			   const ipa::soft::IPAConfigInfo &configInfo)
{
	ASSERT(ipa_ && debayer_);

	int ret = ipa_->configure(configInfo);
	if (ret < 0)
		return ret;

	return debayer_->configure(inputCfg, outputCfgs, ccmEnabled_);
}

/**
 * \brief Export the buffers from the Software ISP
 * \param[in] stream Output stream exporting the buffers
 * \param[in] count Number of buffers to allocate
 * \param[out] buffers Vector to store the allocated buffers
 * \return The number of allocated buffers on success or a negative error code
 * otherwise
 */
int SoftwareIsp::exportBuffers(const Stream *stream, unsigned int count,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	ASSERT(debayer_ != nullptr);

	/* single output for now */
	if (stream == nullptr)
		return -EINVAL;

	return dmaHeap_.exportBuffers(count, { debayer_->frameSize() }, buffers);
}

/**
 * \brief Queue a request and process the control list from the application
 * \param[in] frame The number of the frame which will be processed next
 * \param[in] controls The controls for the \a frame
 */
void SoftwareIsp::queueRequest(const uint32_t frame, const ControlList &controls)
{
	ipa_->queueRequest(frame, controls);
}

/**
 * \brief Queue buffers to Software ISP
 * \param[in] frame The frame number
 * \param[in] input The input framebuffer
 * \param[in] outputs The container holding the output stream pointers and
 * their respective frame buffer outputs
 * \return 0 on success, a negative errno on failure
 */
int SoftwareIsp::queueBuffers(uint32_t frame, FrameBuffer *input,
			      const std::map<const Stream *, FrameBuffer *> &outputs)
{
	/*
	 * Validate the outputs as a sanity check: at least one output is
	 * required, all outputs must reference a valid stream.
	 */
	if (outputs.empty())
		return -EINVAL;

	/* We only support a single stream for now. */
	if (outputs.size() != 1)
		return -EINVAL;

	for (auto [stream, buffer] : outputs) {
		if (!buffer)
			return -EINVAL;
	}

	queuedInputBuffers_.push_back(input);

	for (auto iter = outputs.begin(); iter != outputs.end(); iter++) {
		FrameBuffer *const buffer = iter->second;
		queuedOutputBuffers_.push_back(buffer);
		process(frame, input, buffer);
	}

	return 0;
}

/**
 * \brief Starts the Software ISP streaming operation
 * \return 0 on success, any other value indicates an error
 */
int SoftwareIsp::start()
{
	int ret = ipa_->start();
	if (ret)
		return ret;

	ispWorkerThread_.start();
	return 0;
}

/**
 * \brief Stops the Software ISP streaming operation
 *
 * All pending buffers are returned back as canceled before this function
 * returns.
 */
void SoftwareIsp::stop()
{
	ispWorkerThread_.exit();
	ispWorkerThread_.wait();

	Thread::current()->dispatchMessages(Message::Type::InvokeMessage, this);

	ipa_->stop();

	for (auto buffer : queuedOutputBuffers_) {
		FrameMetadata &metadata = buffer->_d()->metadata();
		metadata.status = FrameMetadata::FrameCancelled;
		outputBufferReady.emit(buffer);
	}
	queuedOutputBuffers_.clear();

	for (auto buffer : queuedInputBuffers_) {
		FrameMetadata &metadata = buffer->_d()->metadata();
		metadata.status = FrameMetadata::FrameCancelled;
		inputBufferReady.emit(buffer);
	}
	queuedInputBuffers_.clear();
}

/**
 * \brief Passes the input framebuffer to the ISP worker to process
 * \param[in] frame The frame number
 * \param[in] input The input framebuffer
 * \param[out] output The framebuffer to write the processed frame to
 */
void SoftwareIsp::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output)
{
	ipa_->computeParams(frame);
	debayer_->invokeMethod(&DebayerCpu::process,
			       ConnectionTypeQueued, frame, input, output, debayerParams_);
}

void SoftwareIsp::saveIspParams()
{
	debayerParams_ = *sharedParams_;
}

void SoftwareIsp::setSensorCtrls(const ControlList &sensorControls)
{
	setSensorControls.emit(sensorControls);
}

void SoftwareIsp::statsReady(uint32_t frame, uint32_t bufferId)
{
	ispStatsReady.emit(frame, bufferId);
}

void SoftwareIsp::inputReady(FrameBuffer *input)
{
	ASSERT(queuedInputBuffers_.front() == input);
	queuedInputBuffers_.pop_front();
	inputBufferReady.emit(input);
}

void SoftwareIsp::outputReady(FrameBuffer *output)
{
	ASSERT(queuedOutputBuffers_.front() == output);
	queuedOutputBuffers_.pop_front();
	outputBufferReady.emit(output);
}

} /* namespace libcamera */
