/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Test:
 * - Multiple reconfigurations of the Camera without stopping the CameraManager
 * - Validate there are no file descriptor leaks when using IPC
 */

#include <dirent.h>
#include <fstream>
#include <iostream>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/file.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include <libcamera/framebuffer_allocator.h>

#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

namespace {

class CameraReconfigure : public CameraTest, public Test
{
public:
	/* Initialize CameraTest with isolated IPA */
	CameraReconfigure()
		: CameraTest(kCamId_, true)
	{
	}

private:
	static constexpr const char *kCamId_ = "platform/vimc.0 Sensor B";
	static constexpr const char *kIpaProxyName_ = "vimc_ipa_proxy";
	static constexpr unsigned int kNumOfReconfigures_ = 10;

	void requestComplete(Request *request)
	{
		if (request->status() != Request::RequestComplete)
			return;

		const Request::BufferMap &buffers = request->buffers();

		const Stream *stream = buffers.begin()->first;
		FrameBuffer *buffer = buffers.begin()->second;

		/* Reuse the request and re-queue it with the same buffers. */
		request->reuse();
		request->addBuffer(stream, buffer);
		camera_->queueRequest(request);
	}

	int startAndStop()
	{
		StreamConfiguration &cfg = config_->at(0);

		if (camera_->acquire()) {
			cerr << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		if (camera_->configure(config_.get())) {
			cerr << "Failed to set default configuration" << endl;
			return TestFail;
		}

		Stream *stream = cfg.stream();

		/*
		 * The configuration is consistent so we can re-use the
		 * same buffer allocation for each run.
		 */
		if (!allocated_) {
			int ret = allocator_->allocate(stream);
			if (ret < 0) {
				cerr << "Failed to allocate buffers" << endl;
				return TestFail;
			}
			allocated_ = true;
		}

		for (const unique_ptr<FrameBuffer> &buffer : allocator_->buffers(stream)) {
			unique_ptr<Request> request = camera_->createRequest();
			if (!request) {
				cerr << "Failed to create request" << endl;
				return TestFail;
			}

			if (request->addBuffer(stream, buffer.get())) {
				cerr << "Failed to associate buffer with request" << endl;
				return TestFail;
			}

			requests_.push_back(std::move(request));
		}

		camera_->requestCompleted.connect(this, &CameraReconfigure::requestComplete);

		if (camera_->start()) {
			cerr << "Failed to start camera" << endl;
			return TestFail;
		}

		for (unique_ptr<Request> &request : requests_) {
			if (camera_->queueRequest(request.get())) {
				cerr << "Failed to queue request" << endl;
				return TestFail;
			}
		}

		EventDispatcher *dispatcher = Thread::current()->eventDispatcher();

		Timer timer;
		timer.start(100ms);
		while (timer.isRunning())
			dispatcher->processEvents();

		if (camera_->stop()) {
			cerr << "Failed to stop camera" << endl;
			return TestFail;
		}

		if (camera_->release()) {
			cerr << "Failed to release camera" << endl;
			return TestFail;
		}

		camera_->requestCompleted.disconnect(this);

		requests_.clear();

		return 0;
	}

	int fdsOpen(pid_t pid)
	{
		string proxyFdPath = "/proc/" + to_string(pid) + "/fd";
		DIR *dir;
		struct dirent *ptr;
		unsigned int openFds = 0;

		dir = opendir(proxyFdPath.c_str());
		if (dir == nullptr) {
			int err = errno;
			cerr << "Error opening " << proxyFdPath << ": "
			     << strerror(-err) << endl;
			return 0;
		}

		while ((ptr = readdir(dir)) != nullptr) {
			if ((strcmp(ptr->d_name, ".") == 0) ||
			    (strcmp(ptr->d_name, "..") == 0))
				continue;

			openFds++;
		}
		closedir(dir);

		return openFds;
	}

	pid_t findProxyPid()
	{
		string proxyPid;
		string proxyName(kIpaProxyName_);
		DIR *dir;
		struct dirent *ptr;

		dir = opendir("/proc");
		while ((ptr = readdir(dir)) != nullptr) {
			if (ptr->d_type != DT_DIR)
				continue;

			string pname("/proc/" + string(ptr->d_name) + "/comm");
			if (File::exists(pname)) {
				ifstream pfile(pname.c_str());
				string comm;
				getline(pfile, comm);
				pfile.close();

				proxyPid = comm == proxyName ? string(ptr->d_name) : "";
			}

			if (!proxyPid.empty())
				break;
		}
		closedir(dir);

		if (!proxyPid.empty())
			return atoi(proxyPid.c_str());

		return -1;
	}

	int init() override
	{
		if (status_ != TestPass)
			return status_;

		config_ = camera_->generateConfiguration({ StreamRole::StillCapture });
		if (!config_ || config_->size() != 1) {
			cerr << "Failed to generate default configuration" << endl;
			return TestFail;
		}

		allocator_ = make_unique<FrameBufferAllocator>(camera_);
		allocated_ = false;

		return TestPass;
	}

	int run() override
	{
		unsigned int openFdsAtStart = 0;
		unsigned int openFds = 0;

		pid_t proxyPid = findProxyPid();
		if (proxyPid < 0) {
			cerr << "Cannot find " << kIpaProxyName_
			     << " pid, exiting" << endl;
			return TestFail;
		}

		openFdsAtStart = fdsOpen(proxyPid);
		for (unsigned int i = 0; i < kNumOfReconfigures_; i++) {
			startAndStop();
			openFds = fdsOpen(proxyPid);
			if (openFds == 0) {
				cerr << "No open fds found whereas "
				     << "open fds at start: " << openFdsAtStart
				     << endl;
				return TestFail;
			}

			if (openFds != openFdsAtStart) {
				cerr << "Leaking fds for " << kIpaProxyName_
				     << " - Open fds: " << openFds << " vs "
				     << "Open fds at start: " << openFdsAtStart
				     << endl;
				return TestFail;
			}
		}

		return TestPass;
	}

	bool allocated_;

	vector<unique_ptr<Request>> requests_;

	unique_ptr<CameraConfiguration> config_;
	unique_ptr<FrameBufferAllocator> allocator_;
};

} /* namespace */

TEST_REGISTER(CameraReconfigure)
