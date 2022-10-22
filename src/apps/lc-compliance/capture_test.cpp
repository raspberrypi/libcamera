/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 * Copyright (C) 2021, Collabora Ltd.
 *
 * capture_test.cpp - Test camera capture
 */

#include <iostream>

#include <gtest/gtest.h>

#include "environment.h"
#include "simple_capture.h"

using namespace libcamera;

const std::vector<int> NUMREQUESTS = { 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };
const std::vector<StreamRole> ROLES = {
	StreamRole::Raw,
	StreamRole::StillCapture,
	StreamRole::VideoRecording,
	StreamRole::Viewfinder
};

class SingleStream : public testing::TestWithParam<std::tuple<StreamRole, int>>
{
public:
	static std::string nameParameters(const testing::TestParamInfo<SingleStream::ParamType> &info);

protected:
	void SetUp() override;
	void TearDown() override;

	std::shared_ptr<Camera> camera_;
};

/*
 * We use gtest's SetUp() and TearDown() instead of constructor and destructor
 * in order to be able to assert on them.
 */
void SingleStream::SetUp()
{
	Environment *env = Environment::get();

	camera_ = env->cm()->get(env->cameraId());

	ASSERT_EQ(camera_->acquire(), 0);
}

void SingleStream::TearDown()
{
	if (!camera_)
		return;

	camera_->release();
	camera_.reset();
}

std::string SingleStream::nameParameters(const testing::TestParamInfo<SingleStream::ParamType> &info)
{
	std::map<StreamRole, std::string> rolesMap = {
		{ StreamRole::Raw, "Raw" },
		{ StreamRole::StillCapture, "StillCapture" },
		{ StreamRole::VideoRecording, "VideoRecording" },
		{ StreamRole::Viewfinder, "Viewfinder" }
	};

	std::string roleName = rolesMap[std::get<0>(info.param)];
	std::string numRequestsName = std::to_string(std::get<1>(info.param));

	return roleName + "_" + numRequestsName;
}

/*
 * Test single capture cycles
 *
 * Makes sure the camera completes the exact number of requests queued. Example
 * failure is a camera that completes less requests than the number of requests
 * queued.
 */
TEST_P(SingleStream, Capture)
{
	auto [role, numRequests] = GetParam();

	SimpleCaptureBalanced capture(camera_);

	capture.configure(role);

	capture.capture(numRequests);
}

/*
 * Test multiple start/stop cycles
 *
 * Makes sure the camera supports multiple start/stop cycles. Example failure is
 * a camera that does not clean up correctly in its error path but is only
 * tested by single-capture applications.
 */
TEST_P(SingleStream, CaptureStartStop)
{
	auto [role, numRequests] = GetParam();
	unsigned int numRepeats = 3;

	SimpleCaptureBalanced capture(camera_);

	capture.configure(role);

	for (unsigned int starts = 0; starts < numRepeats; starts++)
		capture.capture(numRequests);
}

/*
 * Test unbalanced stop
 *
 * Makes sure the camera supports a stop with requests queued. Example failure
 * is a camera that does not handle cancelation of buffers coming back from the
 * video device while stopping.
 */
TEST_P(SingleStream, UnbalancedStop)
{
	auto [role, numRequests] = GetParam();

	SimpleCaptureUnbalanced capture(camera_);

	capture.configure(role);

	capture.capture(numRequests);
}

INSTANTIATE_TEST_SUITE_P(CaptureTests,
			 SingleStream,
			 testing::Combine(testing::ValuesIn(ROLES),
					  testing::ValuesIn(NUMREQUESTS)),
			 SingleStream::nameParameters);
