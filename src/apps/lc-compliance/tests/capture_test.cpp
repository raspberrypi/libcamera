/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 * Copyright (C) 2021, Collabora Ltd.
 *
 * Test camera capture
 */

#include "capture.h"

#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include "test_base.h"

namespace {

using namespace libcamera;

class SimpleCapture : public testing::TestWithParam<std::tuple<std::vector<StreamRole>, int>>, public CameraHolder
{
public:
	static std::string nameParameters(const testing::TestParamInfo<SimpleCapture::ParamType> &info);

protected:
	void SetUp() override;
	void TearDown() override;
};

/*
 * We use gtest's SetUp() and TearDown() instead of constructor and destructor
 * in order to be able to assert on them.
 */
void SimpleCapture::SetUp()
{
	acquireCamera();
}

void SimpleCapture::TearDown()
{
	releaseCamera();
}

std::string SimpleCapture::nameParameters(const testing::TestParamInfo<SimpleCapture::ParamType> &info)
{
	const auto &[roles, numRequests] = info.param;
	std::ostringstream ss;

	for (StreamRole r : roles)
		ss << r << '_';

	ss << '_' << numRequests;

	return ss.str();
}

/*
 * Test single capture cycles
 *
 * Makes sure the camera completes the exact number of requests queued. Example
 * failure is a camera that completes less requests than the number of requests
 * queued.
 */
TEST_P(SimpleCapture, Capture)
{
	const auto &[roles, numRequests] = GetParam();

	Capture capture(camera_);

	capture.configure(roles);

	capture.run(numRequests, numRequests);
}

/*
 * Test multiple start/stop cycles
 *
 * Makes sure the camera supports multiple start/stop cycles. Example failure is
 * a camera that does not clean up correctly in its error path but is only
 * tested by single-capture applications.
 */
TEST_P(SimpleCapture, CaptureStartStop)
{
	const auto &[roles, numRequests] = GetParam();
	unsigned int numRepeats = 3;

	Capture capture(camera_);

	capture.configure(roles);

	for (unsigned int starts = 0; starts < numRepeats; starts++)
		capture.run(numRequests, numRequests);
}

/*
 * Test unbalanced stop
 *
 * Makes sure the camera supports a stop with requests queued. Example failure
 * is a camera that does not handle cancelation of buffers coming back from the
 * video device while stopping.
 */
TEST_P(SimpleCapture, UnbalancedStop)
{
	const auto &[roles, numRequests] = GetParam();

	Capture capture(camera_);

	capture.configure(roles);

	capture.run(numRequests);
}

const int NUMREQUESTS[] = { 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };

const std::vector<StreamRole> SINGLEROLES[] = {
	{ StreamRole::Raw, },
	{ StreamRole::StillCapture, },
	{ StreamRole::VideoRecording, },
	{ StreamRole::Viewfinder, },
};

const std::vector<StreamRole> MULTIROLES[] = {
	{ StreamRole::Raw, StreamRole::StillCapture },
	{ StreamRole::Raw, StreamRole::VideoRecording },
	{ StreamRole::StillCapture, StreamRole::VideoRecording },
	{ StreamRole::VideoRecording, StreamRole::VideoRecording },
};

INSTANTIATE_TEST_SUITE_P(SingleStream,
			 SimpleCapture,
			 testing::Combine(testing::ValuesIn(SINGLEROLES),
					  testing::ValuesIn(NUMREQUESTS)),
			 SimpleCapture::nameParameters);

INSTANTIATE_TEST_SUITE_P(MultiStream,
			 SimpleCapture,
			 testing::Combine(testing::ValuesIn(MULTIROLES),
					  testing::ValuesIn(NUMREQUESTS)),
			 SimpleCapture::nameParameters);

} /* namespace */
