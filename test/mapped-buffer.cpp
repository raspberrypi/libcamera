/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * libcamera internal MappedBuffer tests
 */

#include <iostream>

#include <libcamera/framebuffer_allocator.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "camera_test.h"
#include "test.h"

using namespace libcamera;
using namespace std;

namespace {

class MappedBufferTest : public CameraTest, public Test
{
public:
	MappedBufferTest()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	int init() override
	{
		if (status_ != TestPass)
			return status_;

		config_ = camera_->generateConfiguration({ StreamRole::VideoRecording });
		if (!config_ || config_->size() != 1) {
			cout << "Failed to generate default configuration" << endl;
			return TestFail;
		}

		allocator_ = new FrameBufferAllocator(camera_);

		StreamConfiguration &cfg = config_->at(0);

		if (camera_->acquire()) {
			cout << "Failed to acquire the camera" << endl;
			return TestFail;
		}

		if (camera_->configure(config_.get())) {
			cout << "Failed to set default configuration" << endl;
			return TestFail;
		}

		stream_ = cfg.stream();

		int ret = allocator_->allocate(stream_);
		if (ret < 0)
			return TestFail;

		return TestPass;
	}

	void cleanup() override
	{
		delete allocator_;
	}

	int run() override
	{
		const std::unique_ptr<FrameBuffer> &buffer = allocator_->buffers(stream_).front();
		std::vector<MappedBuffer> maps;

		MappedFrameBuffer map(buffer.get(), MappedFrameBuffer::MapFlag::Read);
		if (!map.isValid()) {
			cout << "Failed to successfully map buffer" << endl;
			return TestFail;
		}

		/* Make sure we can move it. */
		maps.emplace_back(std::move(map));

		/* But copying is prevented, it would cause double-unmap. */
		// MappedFrameBuffer map_copy = map;

		/* Local map should be invalid (after move). */
		if (map.isValid()) {
			cout << "Post-move map should not be valid" << endl;
			return TestFail;
		}

		/* Test for multiple successful maps on the same buffer. */
		MappedFrameBuffer write_map(buffer.get(), MappedFrameBuffer::MapFlag::Write);
		if (!write_map.isValid()) {
			cout << "Failed to map write buffer" << endl;
			return TestFail;
		}

		MappedFrameBuffer rw_map(buffer.get(), MappedFrameBuffer::MapFlag::ReadWrite);
		if (!rw_map.isValid()) {
			cout << "Failed to map RW buffer" << endl;
			return TestFail;
		}

		return TestPass;
	}

private:
	std::unique_ptr<CameraConfiguration> config_;
	FrameBufferAllocator *allocator_;
	Stream *stream_;
};

} /* namespace */

TEST_REGISTER(MappedBufferTest)
