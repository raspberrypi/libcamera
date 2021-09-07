/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_serialization.cpp - Serialize and deserialize controls
 */

#include <iostream>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>

#include "libcamera/internal/byte_stream_buffer.h"
#include "libcamera/internal/control_serializer.h"

#include "serialization_test.h"
#include "test.h"

using namespace std;
using namespace libcamera;

class ControlSerializationTest : public SerializationTest
{
protected:
	int init() override
	{
		return status_;
	}

	int run() override
	{
		ControlSerializer serializer(ControlSerializer::Role::Proxy);
		ControlSerializer deserializer(ControlSerializer::Role::Worker);

		std::vector<uint8_t> infoData;
		std::vector<uint8_t> listData;

		size_t size;
		int ret;

		/* Create a control list with three controls. */
		const ControlInfoMap &infoMap = camera_->controls();
		ControlList list(infoMap);

		list.set(controls::Brightness, 0.5f);
		list.set(controls::Contrast, 1.2f);
		list.set(controls::Saturation, 0.2f);

		/*
		 * Serialize the control list, this should fail as the control
		 * info map hasn't been serialized.
		 */
		size = serializer.binarySize(list);
		listData.resize(size);
		ByteStreamBuffer buffer(listData.data(), listData.size());

		ret = serializer.serialize(list, buffer);
		if (!ret) {
			cerr << "List serialization without info map should have failed"
			     << endl;
			return TestFail;
		}

		if (buffer.overflow() || buffer.offset()) {
			cerr << "Failed list serialization modified the buffer"
			     << endl;
			return TestFail;
		}

		/* Serialize the control info map. */
		size = serializer.binarySize(infoMap);
		infoData.resize(size);
		buffer = ByteStreamBuffer(infoData.data(), infoData.size());

		ret = serializer.serialize(infoMap, buffer);
		if (ret < 0) {
			cerr << "Failed to serialize ControlInfoMap" << endl;
			return TestFail;
		}

		if (buffer.overflow()) {
			cerr << "Overflow when serializing ControlInfoMap" << endl;
			return TestFail;
		}

		/* Serialize the control list, this should now succeed. */
		size = serializer.binarySize(list);
		listData.resize(size);
		buffer = ByteStreamBuffer(listData.data(), listData.size());

		ret = serializer.serialize(list, buffer);
		if (ret) {
			cerr << "Failed to serialize ControlList" << endl;
			return TestFail;
		}

		if (buffer.overflow()) {
			cerr << "Overflow when serializing ControlList" << endl;
			return TestFail;
		}

		/*
		 * Deserialize the control list, this should fail as the control
		 * info map hasn't been deserialized.
		 */
		buffer = ByteStreamBuffer(const_cast<const uint8_t *>(listData.data()),
					  listData.size());

		ControlList newList = deserializer.deserialize<ControlList>(buffer);
		if (!newList.empty()) {
			cerr << "List deserialization without info map should have failed"
			     << endl;
			return TestFail;
		}

		if (buffer.overflow()) {
			cerr << "Failed list deserialization modified the buffer"
			     << endl;
			return TestFail;
		}

		/* Deserialize the control info map and verify the contents. */
		buffer = ByteStreamBuffer(const_cast<const uint8_t *>(infoData.data()),
					  infoData.size());

		ControlInfoMap newInfoMap = deserializer.deserialize<ControlInfoMap>(buffer);
		if (newInfoMap.empty()) {
			cerr << "Failed to deserialize ControlInfoMap" << endl;
			return TestFail;
		}

		if (buffer.overflow()) {
			cerr << "Overflow when deserializing ControlInfoMap" << endl;
			return TestFail;
		}

		if (!equals(infoMap, newInfoMap)) {
			cerr << "Deserialized map doesn't match original" << endl;
			return TestFail;
		}

		/* Make sure control limits looked up by id are not changed. */
		const ControlInfo &newLimits = newInfoMap.at(&controls::Brightness);
		const ControlInfo &initialLimits = infoMap.at(&controls::Brightness);
		if (newLimits.min() != initialLimits.min() ||
		    newLimits.max() != initialLimits.max()) {
			cerr << "The brightness control limits have changed" << endl;
			return TestFail;
		}

		/* Deserialize the control list and verify the contents. */
		buffer = ByteStreamBuffer(const_cast<const uint8_t *>(listData.data()),
					  listData.size());

		newList = deserializer.deserialize<ControlList>(buffer);
		if (newList.empty()) {
			cerr << "Failed to deserialize ControlList" << endl;
			return TestFail;
		}

		if (buffer.overflow()) {
			cerr << "Overflow when deserializing ControlList" << endl;
			return TestFail;
		}

		if (!equals(list, newList)) {
			cerr << "Deserialized list doesn't match original" << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(ControlSerializationTest)
