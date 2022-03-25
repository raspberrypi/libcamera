/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * Test the buffer cache different operation modes
 */

#include <iostream>
#include <random>
#include <vector>

#include <libcamera/formats.h>
#include <libcamera/stream.h>

#include "buffer_source.h"

#include "test.h"

using namespace libcamera;

namespace {

class BufferCacheTest : public Test
{
public:
	/*
	 * Test that a cache with the same size as there are buffers results in
	 * a sequential run over; 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, ...
	 *
	 * The test is only valid when the cache size is as least as big as the
	 * number of buffers.
	 */
	int testSequential(V4L2BufferCache *cache,
			   const std::vector<std::unique_ptr<FrameBuffer>> &buffers)
	{
		for (unsigned int i = 0; i < buffers.size() * 100; i++) {
			int nBuffer = i % buffers.size();
			int index = cache->get(*buffers[nBuffer].get());

			if (index != nBuffer) {
				std::cout << "Expected index " << nBuffer
					  << " got " << index << std::endl;
				return TestFail;
			}

			cache->put(index);
		}

		return TestPass;
	}

	/*
	 * Test that randomly putting buffers to the cache always results in a
	 * valid index.
	 */
	int testRandom(V4L2BufferCache *cache,
		       const std::vector<std::unique_ptr<FrameBuffer>> &buffers)
	{
		std::uniform_int_distribution<> dist(0, buffers.size() - 1);

		for (unsigned int i = 0; i < buffers.size() * 100; i++) {
			int nBuffer = dist(generator_);
			int index = cache->get(*buffers[nBuffer].get());

			if (index < 0) {
				std::cout << "Failed lookup from cache"
					  << std::endl;
				return TestFail;
			}

			cache->put(index);
		}

		return TestPass;
	}

	/*
	 * Test that using a buffer more frequently keeps it hot in the cache at
	 * all times.
	 */
	int testHot(V4L2BufferCache *cache,
		    const std::vector<std::unique_ptr<FrameBuffer>> &buffers,
		    unsigned int hotFrequency)
	{
		/* Run the random test on the cache to make it messy. */
		if (testRandom(cache, buffers) != TestPass)
			return TestFail;

		std::uniform_int_distribution<> dist(0, buffers.size() - 1);

		/* Pick a hot buffer at random and store its index. */
		int hotBuffer = dist(generator_);
		int hotIndex = cache->get(*buffers[hotBuffer].get());
		cache->put(hotIndex);

		/*
		 * Queue hot buffer at the requested frequency and make sure
		 * it stays hot.
		 */
		for (unsigned int i = 0; i < buffers.size() * 100; i++) {
			int nBuffer, index;
			bool hotQueue = i % hotFrequency == 0;

			if (hotQueue)
				nBuffer = hotBuffer;
			else
				nBuffer = dist(generator_);

			index = cache->get(*buffers[nBuffer].get());

			if (index < 0) {
				std::cout << "Failed lookup from cache"
					  << std::endl;
				return TestFail;
			}

			if (hotQueue && index != hotIndex) {
				std::cout << "Hot buffer got cold"
					  << std::endl;
				return TestFail;
			}

			cache->put(index);
		}

		return TestPass;
	}

	int testIsEmpty(const std::vector<std::unique_ptr<FrameBuffer>> &buffers)
	{
		V4L2BufferCache cache(buffers.size());

		if (!cache.isEmpty())
			return TestFail;

		for (auto const &buffer : buffers) {
			FrameBuffer &b = *buffer.get();
			cache.get(b);
		}

		if (cache.isEmpty())
			return TestFail;

		unsigned int i;
		for (i = 0; i < buffers.size() - 1; i++)
			cache.put(i);

		if (cache.isEmpty())
			return TestFail;

		cache.put(i);
		if (!cache.isEmpty())
			return TestFail;

		return TestPass;
	}

	int init() override
	{
		std::random_device rd;
		unsigned int seed = rd();

		std::cout << "Random seed is " << seed << std::endl;

		generator_.seed(seed);

		return TestPass;
	}

	int run() override
	{
		const unsigned int numBuffers = 8;

		StreamConfiguration cfg;
		cfg.pixelFormat = formats::YUYV;
		cfg.size = Size(600, 800);
		cfg.bufferCount = numBuffers;

		BufferSource source;
		int ret = source.allocate(cfg);
		if (ret != TestPass)
			return ret;

		const std::vector<std::unique_ptr<FrameBuffer>> &buffers =
			source.buffers();

		if (buffers.size() != numBuffers) {
			std::cout << "Got " << buffers.size()
				  << " buffers, expected " << numBuffers
				  << std::endl;
			return TestFail;
		}

		/*
		 * Test cache of same size as there are buffers, the cache is
		 * created from a list of buffers and will be pre-populated.
		 */
		V4L2BufferCache cacheFromBuffers(buffers);

		if (testSequential(&cacheFromBuffers, buffers) != TestPass)
			return TestFail;

		if (testRandom(&cacheFromBuffers, buffers) != TestPass)
			return TestFail;

		if (testHot(&cacheFromBuffers, buffers, numBuffers) != TestPass)
			return TestFail;

		/*
		 * Test cache of same size as there are buffers, the cache is
		 * not pre-populated.
		 */
		V4L2BufferCache cacheFromNumbers(numBuffers);

		if (testSequential(&cacheFromNumbers, buffers) != TestPass)
			return TestFail;

		if (testRandom(&cacheFromNumbers, buffers) != TestPass)
			return TestFail;

		if (testHot(&cacheFromNumbers, buffers, numBuffers) != TestPass)
			return TestFail;

		/*
		 * Test cache half the size of number of buffers used, the cache
		 * is not pre-populated.
		 */
		V4L2BufferCache cacheHalf(numBuffers / 2);

		if (testRandom(&cacheHalf, buffers) != TestPass)
			return TestFail;

		if (testHot(&cacheHalf, buffers, numBuffers / 2) != TestPass)
			return TestFail;

		/*
		 * Test that the isEmpty function reports the correct result at
		 * various levels of cache fullness.
		 */
		if (testIsEmpty(buffers) != TestPass)
			return TestFail;

		return TestPass;
	}

private:
	std::mt19937 generator_;
};

} /* namespace */

TEST_REGISTER(BufferCacheTest)
