/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2023, Raspberry Pi Ltd
 *
 * Pipeline handler base class for Raspberry Pi devices
 */

#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include <libcamera/controls.h>
#include <libcamera/request.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/camera.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/clock_recovery.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/media_object.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/v4l2_videodevice.h"
#include "libcamera/internal/yaml_parser.h"

#include <libcamera/ipa/raspberrypi_ipa_interface.h>
#include <libcamera/ipa/raspberrypi_ipa_proxy.h>

#include "delayed_controls.h"
#include "rpi_stream.h"

using namespace std::chrono_literals;

namespace libcamera {

namespace RPi {

/* Map of mbus codes to supported sizes reported by the sensor. */
using SensorFormats = std::map<unsigned int, std::vector<Size>>;

class RPiCameraConfiguration;
class CameraData : public Camera::Private
{
public:
	CameraData(PipelineHandler *pipe)
		: Camera::Private(pipe), state_(State::Stopped),
		  startupFrameCount_(0), invalidFrameCount_(0), buffersAllocated_(false)
	{
	}

	virtual ~CameraData()
	{
	}

	virtual CameraConfiguration::Status platformValidate(RPiCameraConfiguration *rpiConfig) const = 0;
	virtual int platformConfigure(const RPiCameraConfiguration *rpiConfig) = 0;
	virtual void platformStart() = 0;
	virtual void platformStop() = 0;

	double scoreFormat(double desired, double actual) const;
	V4L2SubdeviceFormat findBestFormat(const Size &req, unsigned int bitDepth) const;

	void freeBuffers();
	virtual void platformFreeBuffers() = 0;

	bool enumerateVideoDevices(MediaLink *link, const std::string &frontend);

	int loadPipelineConfiguration();
	int loadIPA(ipa::RPi::InitResult *result);
	int configureIPA(const CameraConfiguration *config, ipa::RPi::ConfigResult *result);
	virtual int platformInitIpa(ipa::RPi::InitParams &params) = 0;
	virtual int platformConfigureIpa(ipa::RPi::ConfigParams &params) = 0;

	void metadataReady(const ControlList &metadata);
	void setDelayedControls(const ControlList &controls, uint32_t delayContext);
	void setLensControls(const ControlList &controls);
	void setSensorControls(ControlList &controls);

	Rectangle scaleIspCrop(const Rectangle &ispCrop) const;
	void applyScalerCrop(const ControlList &controls);
	virtual void platformSetIspCrop(unsigned int index, const Rectangle &ispCrop) = 0;

	void cameraTimeout();
	void frameStarted(uint32_t sequence);

	void clearIncompleteRequests();
	void handleStreamBuffer(FrameBuffer *buffer, Stream *stream);
	void handleState();

	virtual V4L2VideoDevice::Formats ispFormats() const = 0;
	virtual V4L2VideoDevice::Formats rawFormats() const = 0;
	virtual V4L2VideoDevice *frontendDevice() = 0;

	virtual int platformPipelineConfigure(const std::unique_ptr<YamlObject> &root) = 0;

	std::unique_ptr<ipa::RPi::IPAProxyRPi> ipa_;

	std::unique_ptr<CameraSensor> sensor_;
	SensorFormats sensorFormats_;

	/* The vector below is just for convenience when iterating over all streams. */
	std::vector<Stream *> streams_;
	/* Stores the ids of the buffers mapped in the IPA. */
	std::unordered_set<unsigned int> bufferIds_;
	/*
	 * Stores a cascade of Video Mux or Bridge devices between the sensor and
	 * Unicam together with media link across the entities.
	 */
	std::vector<std::pair<std::unique_ptr<V4L2Subdevice>, MediaLink *>> bridgeDevices_;

	std::unique_ptr<DelayedControls> delayedCtrls_;
	bool sensorMetadata_;

	/*
	 * All the functions in this class are called from a single calling
	 * thread. So, we do not need to have any mutex to protect access to any
	 * of the variables below.
	 */
	enum class State { Stopped, Idle, Busy, IpaComplete, Error };
	State state_;

	bool isRunning()
	{
		return state_ != State::Stopped && state_ != State::Error;
	}

	std::queue<Request *> requestQueue_;

	/* For handling digital zoom. */
	IPACameraSensorInfo sensorInfo_;

	struct CropParams {
		CropParams(Rectangle ispCrop_, Size ispMinCropSize_, unsigned int ispIndex_)
			: ispCrop(ispCrop_), ispMinCropSize(ispMinCropSize_), ispIndex(ispIndex_)
		{
		}

		/* Crop in ISP (camera mode) pixels */
		Rectangle ispCrop;
		/* Minimum crop size in ISP output pixels */
		Size ispMinCropSize;
		/* Index of the ISP output channel for this crop */
		unsigned int ispIndex;
	};

	/* Mapping of CropParams keyed by the output stream order in CameraConfiguration */
	std::map<unsigned int, CropParams> cropParams_;

	unsigned int startupFrameCount_;
	unsigned int invalidFrameCount_;

	/*
	 * If set, this stores the value that represets a gain of one for
	 * the V4L2_CID_NOTIFY_GAINS control.
	 */
	std::optional<int32_t> notifyGainsUnity_;

	/* Have internal buffers been allocated? */
	bool buffersAllocated_;

	struct Config {
		/*
		 * Override the camera timeout value calculated by the IPA based
		 * on frame durations.
		 */
		unsigned int cameraTimeoutValue;
	};

	Config config_;

	ClockRecovery wallClockRecovery_;

protected:
	void fillRequestMetadata(const ControlList &bufferControls,
				 Request *request);

	virtual void tryRunPipeline() = 0;

private:
	void checkRequestCompleted();
};

class PipelineHandlerBase : public PipelineHandler
{
public:
	PipelineHandlerBase(CameraManager *manager)
		: PipelineHandler(manager)
	{
	}

	virtual ~PipelineHandlerBase()
	{
	}

	static bool isRgb(const PixelFormat &pixFmt);
	static bool isYuv(const PixelFormat &pixFmt);
	static bool isRaw(const PixelFormat &pixFmt);

	static bool updateStreamConfig(StreamConfiguration *stream,
				       const V4L2DeviceFormat &format);
	static V4L2DeviceFormat toV4L2DeviceFormat(const V4L2VideoDevice *dev,
						   const StreamConfiguration *stream);
	static V4L2DeviceFormat toV4L2DeviceFormat(const V4L2VideoDevice *dev,
						   const V4L2SubdeviceFormat &format,
						   BayerFormat::Packing packingReq);

	std::unique_ptr<CameraConfiguration>
	generateConfiguration(Camera *camera, Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, libcamera::Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

protected:
	int registerCamera(std::unique_ptr<RPi::CameraData> &cameraData,
			   MediaDevice *frontent, const std::string &frontendName,
			   MediaDevice *backend, MediaEntity *sensorEntity);

	void mapBuffers(Camera *camera, const BufferMap &buffers, unsigned int mask);

	virtual int platformRegister(std::unique_ptr<CameraData> &cameraData,
				     MediaDevice *unicam, MediaDevice *isp) = 0;

private:
	CameraData *cameraData(Camera *camera)
	{
		return static_cast<CameraData *>(camera->_d());
	}

	int queueAllBuffers(Camera *camera);
	virtual int prepareBuffers(Camera *camera) = 0;
};

class RPiCameraConfiguration final : public CameraConfiguration
{
public:
	RPiCameraConfiguration(const CameraData *data)
		: CameraConfiguration(), data_(data)
	{
	}

	CameraConfiguration::Status validateColorSpaces(ColorSpaceFlags flags);
	Status validate() override;

	/* Cache the combinedTransform_ that will be applied to the sensor */
	Transform combinedTransform_;
	/* The sensor format computed in validate() */
	V4L2SubdeviceFormat sensorFormat_;

	struct StreamParams {
		StreamParams()
			: index(0), cfg(nullptr), dev(nullptr)
		{
		}

		StreamParams(unsigned int index_, StreamConfiguration *cfg_)
			: index(index_), cfg(cfg_), dev(nullptr)
		{
		}

		unsigned int index;
		StreamConfiguration *cfg;
		V4L2VideoDevice *dev;
		V4L2DeviceFormat format;
	};

	std::vector<StreamParams> rawStreams_;
	std::vector<StreamParams> outStreams_;

	/*
	 * Store the colour spaces that all our streams will have. RGB format streams
	 * will have the same colorspace as YUV streams, with YCbCr field cleared and
	 * range set to full.
	 */
	std::optional<ColorSpace> yuvColorSpace_;
	std::optional<ColorSpace> rgbColorSpace_;

private:
	const CameraData *data_;
};

} /* namespace RPi */

} /* namespace libcamera */
