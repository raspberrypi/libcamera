/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_wrappers_test.cpp - Test the IPA interface and context wrappers
 */

#include <fcntl.h>
#include <iostream>
#include <memory>
#include <linux/videodev2.h>
#include <sys/stat.h>
#include <unistd.h>

#include <libcamera/controls.h>
#include <libipa/ipa_interface_wrapper.h>

#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_context_wrapper.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/v4l2_subdevice.h"

#include "test.h"

using namespace libcamera;
using namespace std;

enum Operation {
	Op_init,
	Op_start,
	Op_stop,
	Op_configure,
	Op_mapBuffers,
	Op_unmapBuffers,
	Op_processEvent,
};

class TestIPAInterface : public IPAInterface
{
public:
	TestIPAInterface()
		: sequence_(0)
	{
	}

	int init(const IPASettings &settings) override
	{
		if (settings.configurationFile != "/ipa/configuration/file") {
			cerr << "init(): Invalid configuration file" << endl;
			report(Op_init, TestFail);
			return 0;
		}

		report(Op_init, TestPass);
		return 0;
	}

	int start() override
	{
		report(Op_start, TestPass);
		return 0;
	}

	void stop() override
	{
		report(Op_stop, TestPass);
	}

	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result) override
	{
		/* Verify sensorInfo. */
		if (sensorInfo.outputSize.width != 2560 ||
		    sensorInfo.outputSize.height != 1940) {
			cerr << "configure(): Invalid sensor info size "
			     << sensorInfo.outputSize.toString();
		}

		/* Verify streamConfig. */
		if (streamConfig.size() != 2) {
			cerr << "configure(): Invalid number of streams "
			     << streamConfig.size() << endl;
			return report(Op_configure, TestFail);
		}

		auto iter = streamConfig.find(1);
		if (iter == streamConfig.end()) {
			cerr << "configure(): No configuration for stream 1" << endl;
			return report(Op_configure, TestFail);
		}
		const IPAStream *stream = &iter->second;
		if (stream->pixelFormat != V4L2_PIX_FMT_YUYV ||
		    stream->size != Size{ 1024, 768 }) {
			cerr << "configure(): Invalid configuration for stream 1" << endl;
			return report(Op_configure, TestFail);
		}

		iter = streamConfig.find(2);
		if (iter == streamConfig.end()) {
			cerr << "configure(): No configuration for stream 2" << endl;
			return report(Op_configure, TestFail);
		}
		stream = &iter->second;
		if (stream->pixelFormat != V4L2_PIX_FMT_NV12 ||
		    stream->size != Size{ 800, 600 }) {
			cerr << "configure(): Invalid configuration for stream 2" << endl;
			return report(Op_configure, TestFail);
		}

		/* Verify entityControls. */
		auto ctrlIter = entityControls.find(42);
		if (ctrlIter == entityControls.end()) {
			cerr << "configure(): Controls not found" << endl;
			return report(Op_configure, TestFail);
		}

		const ControlInfoMap &infoMap = ctrlIter->second;

		if (infoMap.count(V4L2_CID_BRIGHTNESS) != 1 ||
		    infoMap.count(V4L2_CID_CONTRAST) != 1 ||
		    infoMap.count(V4L2_CID_SATURATION) != 1) {
			cerr << "configure(): Invalid control IDs" << endl;
			return report(Op_configure, TestFail);
		}

		report(Op_configure, TestPass);
	}

	void mapBuffers(const std::vector<IPABuffer> &buffers) override
	{
		if (buffers.size() != 2) {
			cerr << "mapBuffers(): Invalid number of buffers "
			     << buffers.size() << endl;
			return report(Op_mapBuffers, TestFail);
		}

		if (buffers[0].id != 10 ||
		    buffers[1].id != 11) {
			cerr << "mapBuffers(): Invalid buffer IDs" << endl;
			return report(Op_mapBuffers, TestFail);
		}

		if (buffers[0].planes.size() != 3 ||
		    buffers[1].planes.size() != 3) {
			cerr << "mapBuffers(): Invalid number of planes" << endl;
			return report(Op_mapBuffers, TestFail);
		}

		if (buffers[0].planes[0].length != 4096 ||
		    buffers[0].planes[1].length != 0 ||
		    buffers[0].planes[2].length != 0 ||
		    buffers[0].planes[0].length != 4096 ||
		    buffers[1].planes[1].length != 4096 ||
		    buffers[1].planes[2].length != 0) {
			cerr << "mapBuffers(): Invalid length" << endl;
			return report(Op_mapBuffers, TestFail);
		}

		if (buffers[0].planes[0].fd.fd() == -1 ||
		    buffers[0].planes[1].fd.fd() != -1 ||
		    buffers[0].planes[2].fd.fd() != -1 ||
		    buffers[0].planes[0].fd.fd() == -1 ||
		    buffers[1].planes[1].fd.fd() == -1 ||
		    buffers[1].planes[2].fd.fd() != -1) {
			cerr << "mapBuffers(): Invalid dmabuf" << endl;
			return report(Op_mapBuffers, TestFail);
		}

		report(Op_mapBuffers, TestPass);
	}

	void unmapBuffers(const std::vector<unsigned int> &ids) override
	{
		if (ids.size() != 2) {
			cerr << "unmapBuffers(): Invalid number of ids "
			     << ids.size() << endl;
			return report(Op_unmapBuffers, TestFail);
		}

		if (ids[0] != 10 || ids[1] != 11) {
			cerr << "unmapBuffers(): Invalid buffer IDs" << endl;
			return report(Op_unmapBuffers, TestFail);
		}

		report(Op_unmapBuffers, TestPass);
	}

	void processEvent(const IPAOperationData &data) override
	{
		/* Verify operation and data. */
		if (data.operation != Op_processEvent) {
			cerr << "processEvent(): Invalid operation "
			     << data.operation << endl;
			return report(Op_processEvent, TestFail);
		}

		if (data.data != std::vector<unsigned int>{ 1, 2, 3, 4 }) {
			cerr << "processEvent(): Invalid data" << endl;
			return report(Op_processEvent, TestFail);
		}

		/* Verify controls. */
		if (data.controls.size() != 1) {
			cerr << "processEvent(): Controls not found" << endl;
			return report(Op_processEvent, TestFail);
		}

		const ControlList &controls = data.controls[0];
		if (controls.get(V4L2_CID_BRIGHTNESS).get<int32_t>() != 10 ||
		    controls.get(V4L2_CID_CONTRAST).get<int32_t>() != 20 ||
		    controls.get(V4L2_CID_SATURATION).get<int32_t>() != 30) {
			cerr << "processEvent(): Invalid controls" << endl;
			return report(Op_processEvent, TestFail);
		}

		report(Op_processEvent, TestPass);
	}

private:
	void report(Operation op, int status)
	{
		IPAOperationData data;
		data.operation = op;
		data.data.resize(1);
		data.data[0] = status;
		queueFrameAction.emit(sequence_++, data);
	}

	unsigned int sequence_;
};

#define INVOKE(method, ...) \
	invoke(&IPAInterface::method, Op_##method, #method, ##__VA_ARGS__)

class IPAWrappersTest : public Test
{
public:
	IPAWrappersTest()
		: subdev_(nullptr), wrapper_(nullptr), sequence_(0), fd_(-1)
	{
	}

protected:
	int init() override
	{
		/* Locate the VIMC Sensor B subdevice. */
		enumerator_ = unique_ptr<DeviceEnumerator>(DeviceEnumerator::create());
		if (!enumerator_) {
			cerr << "Failed to create device enumerator" << endl;
			return TestFail;
		}

		if (enumerator_->enumerate()) {
			cerr << "Failed to enumerate media devices" << endl;
			return TestFail;
		}

		DeviceMatch dm("vimc");
		media_ = enumerator_->search(dm);
		if (!media_) {
			cerr << "No VIMC media device found: skip test" << endl;
			return TestSkip;
		}

		MediaEntity *entity = media_->getEntityByName("Sensor A");
		if (!entity) {
			cerr << "Unable to find media entity 'Sensor A'" << endl;
			return TestFail;
		}

		subdev_ = new V4L2Subdevice(entity);
		if (subdev_->open() < 0) {
			cerr << "Unable to open 'Sensor A' subdevice" << endl;
			return TestFail;
		}

		/* Force usage of the C API as that's what we want to test. */
		int ret = setenv("LIBCAMERA_IPA_FORCE_C_API", "", 1);
		if (ret)
			return TestFail;

		std::unique_ptr<IPAInterface> intf = std::make_unique<TestIPAInterface>();
		wrapper_ = new IPAContextWrapper(new IPAInterfaceWrapper(std::move(intf)));
		wrapper_->queueFrameAction.connect(this, &IPAWrappersTest::queueFrameAction);

		/* Create a file descriptor for the buffer-related operations. */
		fd_ = open("/tmp", O_TMPFILE | O_RDWR, 0600);
		if (fd_ == -1)
			return TestFail;

		ret = ftruncate(fd_, 4096);
		if (ret < 0)
			return TestFail;

		return TestPass;
	}

	int run() override
	{
		int ret;

		/* Test configure(). */
		CameraSensorInfo sensorInfo{
			.model = "sensor",
			.bitsPerPixel = 8,
			.activeAreaSize = { 2576, 1956 },
			.analogCrop = { 8, 8, 2560, 1940 },
			.outputSize = { 2560, 1940 },
			.pixelRate = 96000000,
			.lineLength = 2918,
		};
		std::map<unsigned int, IPAStream> config{
			{ 1, { V4L2_PIX_FMT_YUYV, { 1024, 768 } } },
			{ 2, { V4L2_PIX_FMT_NV12, { 800, 600 } } },
		};
		std::map<unsigned int, const ControlInfoMap &> controlInfo;
		controlInfo.emplace(42, subdev_->controls());
		IPAOperationData ipaConfig;
		ret = INVOKE(configure, sensorInfo, config, controlInfo,
			     ipaConfig, nullptr);
		if (ret == TestFail)
			return TestFail;

		/* Test mapBuffers(). */
		std::vector<IPABuffer> buffers(2);
		buffers[0].planes.resize(3);
		buffers[0].id = 10;
		buffers[0].planes[0].fd = FileDescriptor(fd_);
		buffers[0].planes[0].length = 4096;
		buffers[1].id = 11;
		buffers[1].planes.resize(3);
		buffers[1].planes[0].fd = FileDescriptor(fd_);
		buffers[1].planes[0].length = 4096;
		buffers[1].planes[1].fd = FileDescriptor(fd_);
		buffers[1].planes[1].length = 4096;

		ret = INVOKE(mapBuffers, buffers);
		if (ret == TestFail)
			return TestFail;

		/* Test unmapBuffers(). */
		std::vector<unsigned int> bufferIds = { 10, 11 };
		ret = INVOKE(unmapBuffers, bufferIds);
		if (ret == TestFail)
			return TestFail;

		/* Test processEvent(). */
		IPAOperationData data;
		data.operation = Op_processEvent;
		data.data = { 1, 2, 3, 4 };
		data.controls.emplace_back(subdev_->controls());

		ControlList &controls = data.controls.back();
		controls.set(V4L2_CID_BRIGHTNESS, static_cast<int32_t>(10));
		controls.set(V4L2_CID_CONTRAST, static_cast<int32_t>(20));
		controls.set(V4L2_CID_SATURATION, static_cast<int32_t>(30));

		ret = INVOKE(processEvent, data);
		if (ret == TestFail)
			return TestFail;

		/*
		 * Test init(), start() and stop() last to ensure nothing in the
		 * wrappers or serializer depends on them being called first.
		 */
		IPASettings settings{
			.configurationFile = "/ipa/configuration/file"
		};
		ret = INVOKE(init, settings);
		if (ret == TestFail) {
			cerr << "Failed to run init()";
			return TestFail;
		}

		ret = INVOKE(start);
		if (ret == TestFail) {
			cerr << "Failed to run start()";
			return TestFail;
		}

		ret = INVOKE(stop);
		if (ret == TestFail) {
			cerr << "Failed to run stop()";
			return TestFail;
		}

		return TestPass;
	}

	void cleanup() override
	{
		delete wrapper_;
		delete subdev_;

		if (fd_ != -1)
			close(fd_);
	}

private:
	template<typename T, typename... Args1, typename... Args2>
	int invoke(T (IPAInterface::*func)(Args1...), Operation op,
		   const char *name, Args2... args)
	{
		data_ = IPAOperationData();
		(wrapper_->*func)(args...);

		if (frame_ != sequence_) {
			cerr << "IPAInterface::" << name
			     << "(): invalid frame number " << frame_
			     << ", expected " << sequence_;
			return TestFail;
		}

		sequence_++;

		if (data_.operation != op) {
			cerr << "IPAInterface::" << name
			     << "(): failed to propagate" << endl;
			return TestFail;
		}

		if (data_.data[0] != TestPass) {
			cerr << "IPAInterface::" << name
			     << "(): reported an error" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void queueFrameAction(unsigned int frame, const IPAOperationData &data)
	{
		frame_ = frame;
		data_ = data;
	}

	std::shared_ptr<MediaDevice> media_;
	std::unique_ptr<DeviceEnumerator> enumerator_;
	V4L2Subdevice *subdev_;

	IPAContextWrapper *wrapper_;
	IPAOperationData data_;
	unsigned int sequence_;
	unsigned int frame_;
	int fd_;
};

TEST_REGISTER(IPAWrappersTest)
