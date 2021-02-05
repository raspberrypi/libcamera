/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * single_stream.cpp - Test a single camera stream
 */

#include <iostream>

#include "simple_capture.h"
#include "tests.h"

using namespace libcamera;

Results::Result testRequestBalance(std::shared_ptr<Camera> camera,
				   StreamRole role, unsigned int startCycles,
				   unsigned int numRequests)
{
	SimpleCaptureBalanced capture(camera);

	Results::Result ret = capture.configure(role);
	if (ret.first != Results::Pass)
		return ret;

	for (unsigned int starts = 0; starts < startCycles; starts++) {
		ret = capture.capture(numRequests);
		if (ret.first != Results::Pass)
			return ret;
	}

	return { Results::Pass, "Balanced capture of " +
		std::to_string(numRequests) + " requests with " +
		std::to_string(startCycles) + " start cycles" };
}

Results::Result testRequestUnbalance(std::shared_ptr<Camera> camera,
				     StreamRole role, unsigned int numRequests)
{
	SimpleCaptureUnbalanced capture(camera);

	Results::Result ret = capture.configure(role);
	if (ret.first != Results::Pass)
		return ret;

	return capture.capture(numRequests);
}

Results testSingleStream(std::shared_ptr<Camera> camera)
{
	static const std::vector<std::pair<std::string, StreamRole>> roles = {
		{ "raw", Raw },
		{ "still", StillCapture },
		{ "video", VideoRecording },
		{ "viewfinder", Viewfinder },
	};
	static const std::vector<unsigned int> numRequests = { 1, 2, 3, 5, 8, 13, 21, 34, 55, 89 };

	Results results(numRequests.size() * roles.size() * 3);

	for (const auto &role : roles) {
		std::cout << "= Test role " << role.first << std::endl;
		/*
		 * Test single capture cycles
		 *
		 * Makes sure the camera completes the exact number of requests queued.
		 * Example failure is a camera that needs N+M requests queued to
		 * complete N requests to the application.
		 */
		std::cout << "* Test single capture cycles" << std::endl;
		for (unsigned int num : numRequests)
			results.add(testRequestBalance(camera, role.second, 1, num));

		/*
		 * Test multiple start/stop cycles
		 *
		 * Makes sure the camera supports multiple start/stop cycles.
		 * Example failure is a camera that does not clean up correctly in its
		 * error path but is only tested by single-capture applications.
		 */
		std::cout << "* Test multiple start/stop cycles" << std::endl;
		for (unsigned int num : numRequests)
			results.add(testRequestBalance(camera, role.second, 3, num));

		/*
		 * Test unbalanced stop
		 *
		 * Makes sure the camera supports a stop with requests queued.
		 * Example failure is a camera that does not handle cancelation
		 * of buffers coming back from the video device while stopping.
		 */
		std::cout << "* Test unbalanced stop" << std::endl;
		for (unsigned int num : numRequests)
			results.add(testRequestUnbalance(camera, role.second, num));
	}

	return results;
}
