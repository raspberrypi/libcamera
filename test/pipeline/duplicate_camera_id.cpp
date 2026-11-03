/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy.
 */

#include <iostream>
#include <stdlib.h>

#include <libcamera/camera.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/pipeline_handler.h"

#include "test.h"

using namespace libcamera;

namespace {

/*
 * There's no easy way to communicate to our custom test pipeline
 * handler. Use a global here to state and track how many cameras should
 * be constructed.
 */
struct MatchState {
	unsigned int requested;
	unsigned int created;
} matchState;

constexpr const char *cameraId = "NotVeryUniqueCameraId";

} /* namespace */

namespace libcamera {

class TestCameraData : public Camera::Private
{
public:
	TestCameraData(PipelineHandler *pipe)
		: Camera::Private(pipe)
	{
	}
};

class PipelineHandlerTest : public PipelineHandler
{
public:
	PipelineHandlerTest(CameraManager *manager)
		: PipelineHandler(manager)
	{
	}

	std::unique_ptr<CameraConfiguration>
	generateConfiguration(Camera *,
			      std::span<const StreamRole>) override
	{
		return {};
	}

	int configure(Camera *, CameraConfiguration *) override
	{
		return true;
	}

	int exportFrameBuffers(Camera *, Stream *,
			       std::vector<std::unique_ptr<FrameBuffer>> *) override
	{
		return true;
	}

	int start(Camera *, const ControlList *) override
	{
		return false;
	}

	void stopDevice(Camera *) override
	{
	}

	int queueRequestDevice(Camera *, Request *) override
	{
		return false;
	}

	bool match(DeviceEnumerator *) override
	{
		/* Only construct requested number of pipelines */
		if (matchState.created >= matchState.requested)
			return false;

		std::unique_ptr<TestCameraData> data = std::make_unique<TestCameraData>(this);

		std::set<Stream *> streams; /* No streams yet on this camera... */
		std::shared_ptr<Camera> camera = Camera::create(std::move(data), cameraId, streams);

		registerCamera(std::move(camera));

		matchState.created++;

		return true;
	}
};

REGISTER_PIPELINE_HANDLER(PipelineHandlerTest, "test")

} /* namespace libcamera */

namespace {

class CameraIdTest : public Test
{
protected:
	int init() override
	{
		/* Make sure CameraManager only uses our custom test pipeline */
		if (setenv("LIBCAMERA_PIPELINES_MATCH_LIST", "test", 1) != 0) {
			std::cerr << "Failed to set pipeline match list" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	int cameraConstructionTest(unsigned int qty)
	{
		libcamera::CameraManager cm;

		/*
		 * Re-initialise the global state used by the test pipeline
		 * handler when we call cm.start().
		 */
		matchState.created = 0;
		matchState.requested = qty;

		cm.start();

		/*
		 * Even if we try to create multiple cameras, we always expect
		 * only one to succeed.
		 */
		const std::vector<std::shared_ptr<Camera>> &cameras = cm.cameras();
		if (cameras.size() != 1) {
			std::cerr
				<< "Expected 1 camera, got "
				<< cameras.size() << std::endl;
			return TestFail;
		}

		/*
		 * I can't see how it could ever be anything but this here, but
		 * let's check anyway as the intent of the test is that we get
		 * only a single camera with the expected ID.
		 */
		const std::string &id = cameras[0]->id();
		if (id != cameraId) {
			std::cerr
				<< "Expected '" << cameraId
				<< "', got '" << id << "'" << std::endl;
			return TestFail;
		}

		return TestPass;
	}

	int run() override
	{
		int ret;

		/*
		 * Sanity check with a single camera construction first, and
		 * this also helps to run this test multiple times to make sure
		 * we're destroying and reconstructing the CameraManager and
		 * pipeline handler fully.
		 */
		ret = cameraConstructionTest(1);
		if (ret != TestPass)
			return ret;

		/* And now lets attempt duplicated camera constructions */
		ret = cameraConstructionTest(3);
		if (ret != TestPass)
			return ret;

		return TestPass;
	}
};

} /* namespace */

TEST_REGISTER(CameraIdTest)
