/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * ByteStreamBuffer tests
 */

#include <array>
#include <iostream>

#include "libcamera/internal/byte_stream_buffer.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class ByteStreamBufferTest : public Test
{
protected:
	int run()
	{
		/*
		 * gcc 11.1.0 incorrectly raises a maybe-uninitialized warning
		 * when calling data.size() below (if the address sanitizer is
		 * disabled). Silence it by initializing the array.
		 */
		std::array<uint8_t, 100> data = {};
		unsigned int i;
		uint32_t value;
		int ret;

		/*
		 * Write mode.
		 */
		ByteStreamBuffer wbuf(data.data(), data.size());

		if (wbuf.base() != data.data() || wbuf.size() != data.size() ||
		    wbuf.offset() != 0 || wbuf.overflow()) {
			cerr << "Write buffer incorrectly constructed" << endl;
			return TestFail;
		}

		/* Test write. */
		value = 0x12345678;
		ret = wbuf.write(&value);
		if (ret || wbuf.offset() != 4 || wbuf.overflow() ||
		    *reinterpret_cast<uint32_t *>(data.data()) != 0x12345678) {
			cerr << "Write failed on write buffer" << endl;
			return TestFail;
		}

		/* Test write carve out. */
		ByteStreamBuffer wco = wbuf.carveOut(10);
		if (wco.base() != wbuf.base() + 4 || wco.size() != 10 ||
		    wco.offset() != 0 || wco.overflow() || wbuf.offset() != 14 ||
		    wbuf.overflow()) {
			cerr << "Carving out write buffer failed" << endl;
			return TestFail;
		}

		/* Test write on the carved out buffer. */
		value = 0x87654321;
		ret = wco.write(&value);
		if (ret || wco.offset() != 4 || wco.overflow() ||
		    *reinterpret_cast<uint32_t *>(data.data() + 4) != 0x87654321) {
			cerr << "Write failed on carve out buffer" << endl;
			return TestFail;
		}

		if (wbuf.offset() != 14 || wbuf.overflow()) {
			cerr << "Write on carve out buffer modified write buffer" << endl;
			return TestFail;
		}

		/* Test read, this should fail. */
		ret = wbuf.read(&value);
		if (!ret || wbuf.overflow()) {
			cerr << "Read should fail on write buffer" << endl;
			return TestFail;
		}

		/* Test overflow on carved out buffer. */
		for (i = 0; i < 2; ++i) {
			ret = wco.write(&value);
			if (ret < 0)
				break;
		}

		if (i != 1 || !wco.overflow() || !wbuf.overflow()) {
			cerr << "Write on carve out buffer failed to overflow" << endl;
			return TestFail;
		}

		/* Test reinitialization of the buffer. */
		wbuf = ByteStreamBuffer(data.data(), data.size());
		if (wbuf.overflow() || wbuf.base() != data.data() ||
		    wbuf.offset() != 0) {
			cerr << "Write buffer reinitialization failed" << endl;
			return TestFail;
		}

		/*
		 * Read mode.
		 */
		ByteStreamBuffer rbuf(const_cast<const uint8_t *>(data.data()),
				      data.size());

		if (rbuf.base() != data.data() || rbuf.size() != data.size() ||
		    rbuf.offset() != 0 || rbuf.overflow()) {
			cerr << "Read buffer incorrectly constructed" << endl;
			return TestFail;
		}

		/* Test read. */
		value = 0;
		ret = rbuf.read(&value);
		if (ret || rbuf.offset() != 4 || rbuf.overflow() ||
		    value != 0x12345678) {
			cerr << "Write failed on write buffer" << endl;
			return TestFail;
		}

		/* Test read carve out. */
		ByteStreamBuffer rco = rbuf.carveOut(10);
		if (rco.base() != rbuf.base() + 4 || rco.size() != 10 ||
		    rco.offset() != 0 || rco.overflow() || rbuf.offset() != 14 ||
		    rbuf.overflow()) {
			cerr << "Carving out read buffer failed" << endl;
			return TestFail;
		}

		/* Test read on the carved out buffer. */
		value = 0;
		ret = rco.read(&value);
		if (ret || rco.offset() != 4 || rco.overflow() || value != 0x87654321) {
			cerr << "Read failed on carve out buffer" << endl;
			return TestFail;
		}

		if (rbuf.offset() != 14 || rbuf.overflow()) {
			cerr << "Read on carve out buffer modified read buffer" << endl;
			return TestFail;
		}

		/* Test write, this should fail. */
		ret = rbuf.write(&value);
		if (!ret || rbuf.overflow()) {
			cerr << "Write should fail on read buffer" << endl;
			return TestFail;
		}

		/* Test overflow on carved out buffer. */
		for (i = 0; i < 2; ++i) {
			ret = rco.read(&value);
			if (ret < 0)
				break;
		}

		if (i != 1 || !rco.overflow() || !rbuf.overflow()) {
			cerr << "Read on carve out buffer failed to overflow" << endl;
			return TestFail;
		}

		/* Test reinitialization of the buffer. */
		rbuf = ByteStreamBuffer(const_cast<const uint8_t *>(data.data()),
					data.size());
		if (rbuf.overflow() || rbuf.base() != data.data() ||
		    rbuf.offset() != 0) {
			cerr << "Read buffer reinitialization failed" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ByteStreamBufferTest)
