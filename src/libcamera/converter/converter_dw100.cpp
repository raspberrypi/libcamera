/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * DW100 Dewarp Engine integration
 */

#include "libcamera/internal/converter/converter_dw100.h"

#include <linux/dw100.h>

#include <libcamera/base/log.h>

#include <libcamera/geometry.h>
#include <libcamera/stream.h>

#include "libcamera/internal/converter.h"
#include "libcamera/internal/converter/converter_v4l2_m2m.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Converter)

/**
 * \class libcamera::ConverterDW100Module
 * \brief A converter module for the dw100 dewarper
 *
 * This class implements a converter module with direct support for libcamera
 * controls. Functionality wise it closely resembles the libcamera::Converter
 * interface. The main difference is that V4L2 requests are handled internally
 * and it has direct support for libcamera controls.
 */

ConverterDW100Module::ConverterDW100Module(std::shared_ptr<MediaDevice> media)
	: converter_(media), running_(false)
{
	converter_.outputBufferReady.connect(&this->outputBufferReady, &Signal<FrameBuffer *>::emit);
	converter_.inputBufferReady.connect(&this->inputBufferReady, &Signal<FrameBuffer *>::emit);
}

/**
 * \brief Create a ConverterDW100Module
 * \param[in] enumerator The enumerator
 *
 * Static factory function that searches for the dw100 device using the provided
 * \a enumerator. If found, a ConverterDW100Module is instantiated and returned.
 *
 * \return A ConverterDW100Module or null if no converter was found
 */
std::unique_ptr<ConverterDW100Module>
ConverterDW100Module::createModule(DeviceEnumerator *enumerator)
{
	DeviceMatch dwp("dw100");
	dwp.add("dw100-source");
	dwp.add("dw100-sink");

	std::shared_ptr<MediaDevice> dwpMediaDevice = enumerator->search(dwp);
	if (!dwpMediaDevice)
		return {};

	std::unique_ptr<ConverterDW100Module> dwpModule{ new ConverterDW100Module(dwpMediaDevice) };
	if (dwpModule->converter_.isValid())
		return dwpModule;

	LOG(Converter, Warning)
		<< "Found DW100 dewarper " << dwpMediaDevice->deviceNode()
		<< " but invalid";
	return {};
}

/**
 * \brief Initialize the module with configuration data
 * \param[in] params The config parameters
 *
 * This function shall be called from the pipeline handler to initialize the
 * module with the provided parameters.
 *
 * A typical tuning file entry for the dewarper looks like this:
 * \code{.unparsed}
 * modules:
 * - Dewarp:
 *    cm: [
 *      1.0, 0.0, 0.0,
 *      0.0, 1.0, 0.0,
 *      0.0, 0.0, 1.0,
 *    ]
 *    coefficients: [
 *      0,0,0,0,0,
 *    ]
 * \endcode
 *
 * The \a cm and \a coefficients parameters are documented in
 * Dw100VertexMap::setDewarpParams()
 *
 * \sa Dw100VertexMap::setDewarpParams()
 * \return 0 if successful, an error code otherwise
 */
int ConverterDW100Module::init(const YamlObject &params)
{
	DewarpParms dp;

	auto &cm = params["cm"];
	auto &coefficients = params["coefficients"];

	/* If nothing is provided, the dewarper is still functional */
	if (!cm && !coefficients)
		return 0;

	if (!cm) {
		LOG(Converter, Error) << "Dewarp parameters are missing 'cm' value";
		return -EINVAL;
	}

	auto matrix = cm.get<Matrix<double, 3, 3>>();
	if (!matrix) {
		LOG(Converter, Error) << "Failed to load 'cm' value";
		return -EINVAL;
	}

	dp.cm = *matrix;

	if (!coefficients) {
		LOG(Converter, Error) << "Dewarp parameters are missing 'coefficients' value";
		return -EINVAL;
	}

	const auto coeffs = coefficients.getList<double>();
	if (!coeffs) {
		LOG(Converter, Error) << "Dewarp parameters 'coefficients' value is not a list";
		return -EINVAL;
	}
	dp.coeffs = std::move(*coeffs);

	dewarpParams_ = dp;

	return 0;
}

/**
 * \copydoc libcamera::V4L2M2MConverter::configure
 */
int ConverterDW100Module::configure(const StreamConfiguration &inputCfg,
				    const std::vector<std::reference_wrapper<StreamConfiguration>>
					    &outputCfgs)
{
	int ret;

	vertexMaps_.clear();
	ret = converter_.configure(inputCfg, outputCfgs);
	if (ret)
		return ret;

	inputBufferCount_ = inputCfg.bufferCount;

	for (auto &ref : outputCfgs) {
		const auto &outputCfg = ref.get();
		auto &info = vertexMaps_[outputCfg.stream()];
		auto &vertexMap = info.map;
		vertexMap.setInputSize(inputCfg.size);
		vertexMap.setOutputSize(outputCfg.size);
		vertexMap.setSensorCrop(sensorCrop_);

		if (dewarpParams_)
			vertexMap.setDewarpParams(dewarpParams_->cm, dewarpParams_->coeffs);
		info.update = true;
	}

	return 0;
}

/**
 * \copydoc libcamera::V4L2M2MConverter::isConfigured
 */
bool ConverterDW100Module::isConfigured(const Stream *stream) const
{
	return vertexMaps_.find(stream) != vertexMaps_.end();
}

/**
 * \copydoc libcamera::V4L2M2MConverter::adjustInputSize
 */
Size ConverterDW100Module::adjustInputSize(const PixelFormat &pixFmt,
					   const Size &size,
					   Converter::Alignment align)
{
	return converter_.adjustInputSize(pixFmt, size, align);
}

/**
 * \copydoc libcamera::V4L2M2MConverter::adjustOutputSize
 */
Size ConverterDW100Module::adjustOutputSize(const PixelFormat &pixFmt,
					    const Size &size,
					    Converter::Alignment align)
{
	return converter_.adjustOutputSize(pixFmt, size, align);
}

/**
 * \copydoc libcamera::V4L2M2MConverter::exportBuffers
 */
int ConverterDW100Module::exportBuffers(const Stream *stream, unsigned int count,
					std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return converter_.exportBuffers(stream, count, buffers);
}

/**
 * \copydoc libcamera::V4L2M2MConverter::validateOutput
 */
int ConverterDW100Module::validateOutput(StreamConfiguration *cfg,
					 bool *adjusted,
					 Converter::Alignment align)
{
	return converter_.validateOutput(cfg, adjusted, align);
}

/**
 * \brief Queue buffers to converter device
 * \param[in] input The frame buffer to apply the conversion
 * \param[out] outputs The container holding the output stream pointers and
 * their respective frame buffer outputs.
 *
 * This function queues the \a input frame buffer and the output frame buffers
 * contained in \a outputs to the device for processing.
 *
 * Controls are automatically applied to the device before queuing buffers. V4L2
 * requests are used to atomically apply the controls if the kernel supports it.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ConverterDW100Module::queueBuffers(FrameBuffer *input,
				       const std::map<const Stream *, FrameBuffer *> &outputs)
{
	int ret;

	V4L2Request *request = nullptr;
	if (!requests_.empty()) {
		/* If we have requests support, there must be one available */
		ASSERT(!availableRequests_.empty());
		request = availableRequests_.front();
		availableRequests_.pop();
	}

	for (auto &[stream, buffer] : outputs) {
		ret = applyControls(stream, request);
		if (ret) {
			reinitRequest(request);
			return ret;
		}
	}

	ret = converter_.queueBuffers(input, outputs, request);
	if (ret) {
		reinitRequest(request);
		return ret;
	}

	if (!request)
		return 0;

	ret = request->queue();
	if (ret < 0) {
		LOG(Converter, Error) << "Failed to queue dewarp request: -"
				      << strerror(-ret);
		/* Push it back into the queue. */
		reinitRequest(request);
	}

	return ret;
}

/**
 * \copydoc libcamera::V4L2M2MConverter::start
 */
int ConverterDW100Module::start()
{
	int ret;

	if (converter_.supportsRequests()) {
		ret = converter_.allocateRequests(inputBufferCount_,
						  &requests_);
		if (ret < 0) {
			LOG(Converter, Error) << "Failed to allocate requests.";
			return ret;
		}
	}

	for (std::unique_ptr<V4L2Request> &request : requests_) {
		request->requestDone.connect(this, &ConverterDW100Module::reinitRequest);
		availableRequests_.push(request.get());
	}

	/*
	 * Apply controls on all streams, to support older kernels without
	 * request and dynamic vertex map support.
	 */
	for (auto &[stream, info] : vertexMaps_)
		applyControls(stream, nullptr);

	ret = converter_.start();
	if (!ret) {
		running_ = true;
		return 0;
	}

	availableRequests_ = {};
	requests_.clear();
	return ret;
}

/**
 * \copydoc libcamera::V4L2M2MConverter::stop
 */
void ConverterDW100Module::stop()
{
	running_ = false;
	converter_.stop();
	availableRequests_ = {};
	requests_.clear();
}

/**
 * \brief Update the controls
 * \param[in] stream The stream
 * \param[inout] controls The controls info map to update
 *
 * Updated the \a controls map with all the controls and limits provided by this
 * class.
 */
void ConverterDW100Module::updateControlInfos(const Stream *stream, ControlInfoMap::Map &controls)
{
	ControlValue scalerCropDefault = sensorCrop_;

	if (isConfigured(stream)) {
		auto &info = vertexMaps_[stream];
		info.map.applyLimits();
		scalerCropDefault = info.map.effectiveScalerCrop();
	}

	controls[&controls::ScalerCrop] = ControlInfo(Rectangle(sensorCrop_.x, sensorCrop_.y, 1, 1),
						      sensorCrop_, sensorCrop_);

	if (!converter_.supportsRequests())
		LOG(Converter, Warning)
			<< "dw100 kernel driver has no requests support."
			   " Dynamic configuration is not possible.";
}

/**
 * \brief Set libcamera controls
 * \param[in] stream The stream to update
 * \param[in] controls The controls
 *
 * Looks up all supported controls in \a controls and sets them on stream \a
 * stream. The controls will be applied to the device on the next call to
 * queueBuffers().
 */
void ConverterDW100Module::setControls(const Stream *stream, const ControlList &controls)
{
	if (!isConfigured(stream))
		return;

	auto &info = vertexMaps_[stream];
	auto &vertexMap = info.map;

	const auto &crop = controls.get(controls::ScalerCrop);
	if (crop) {
		vertexMap.setScalerCrop(*crop);
		info.update = true;
	}

	if (info.update && running_ && !converter_.supportsRequests())
		LOG(Converter, Error)
			<< "Dynamically setting dw100 specific controls requires"
			   " a dw100 kernel driver with requests support";
}

/**
 * \brief Retrieve updated metadata
 * \param[in] stream The stream
 * \param[in] meta The metadata list
 *
 * This function retrieves the metadata for the provided \a stream and writes it
 * to \a list. It shall be called after queueBuffers().
 */
void ConverterDW100Module::populateMetadata(const Stream *stream, ControlList &meta)
{
	if (!isConfigured(stream))
		return;

	auto &vertexMap = vertexMaps_[stream].map;

	meta.set(controls::ScalerCrop, vertexMap.effectiveScalerCrop());
}

/**
 * \var ConverterDW100Module::inputBufferReady
 * \brief A signal emitted when the input frame buffer completes
 */

/**
 * \var ConverterDW100Module::outputBufferReady
 * \brief A signal emitted on each frame buffer completion of the output queue
 */

/**
 * \brief Set sensor crop rectangle
 * \param[in] rect The crop rectangle
 *
 * Set the sensor crop rectangle to \a rect. This rectangle describes the area
 * covered by the input buffers in sensor coordinates. It is used internally
 * to handle the ScalerCrop control and related metadata.
 */
void ConverterDW100Module::setSensorCrop(const Rectangle &rect)
{
	sensorCrop_ = rect;
	for (auto &[stream, vertexMap] : vertexMaps_) {
		vertexMap.map.setSensorCrop(rect);
		vertexMap.update = true;
	}
}

/**
 * \brief Set transform
 * \param[in] stream The stream
 * \param[in] transform The transform
 *
 * Set the transform that shall be applied by the dewarper on the given stream.
 * As orientation is a property of libcamera::CameraConfiguration, the transform
 * needs to be set at configure time.
 */
void ConverterDW100Module::setTransform(const Stream *stream, const Transform &transform)
{
	if (!isConfigured(stream))
		return;

	vertexMaps_[stream].map.setTransform(transform);
}

void ConverterDW100Module::reinitRequest(V4L2Request *request)
{
	if (!request)
		return;

	request->reinit();
	availableRequests_.push(request);
}

/**
 * \brief Apply the vertex map for a given stream
 * \param[in] stream The stream to update
 * \param[in] request An optional request
 *
 * This function updates the vertex map on the stream \a stream. If \a request
 * is provided, the update happens in that request.
 *
 * \return 0 on success or a negative error code otherwise
 */
int ConverterDW100Module::applyControls(const Stream *stream, const V4L2Request *request)
{
	if (!isConfigured(stream))
		return -EINVAL;

	auto &info = vertexMaps_[stream];
	if (!info.update)
		return 0;

	std::vector<uint32_t> map = info.map.getVertexMap();
	auto value = Span<const int32_t>(reinterpret_cast<const int32_t *>(&map[0]), map.size());

	ControlList ctrls;
	ctrls.set(V4L2_CID_DW100_DEWARPING_16x16_VERTEX_MAP, value);

	int ret = converter_.applyControls(stream, ctrls, request);
	if (!ret)
		info.update = false;

	return ret;
}

} /* namespace libcamera */
