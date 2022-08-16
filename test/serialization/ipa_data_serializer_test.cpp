/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipa_data_serializer_test.cpp - Test serializing/deserializing with IPADataSerializer
 */

#include <algorithm>
#include <cxxabi.h>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <tuple>
#include <unistd.h>
#include <vector>

#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "libcamera/internal/ipa_data_serializer.h"

#include "serialization_test.h"
#include "test.h"

using namespace std;
using namespace libcamera;

static const ControlInfoMap Controls = ControlInfoMap({
		{ &controls::AeEnable, ControlInfo(false, true) },
		{ &controls::ExposureTime, ControlInfo(0, 999999) },
		{ &controls::AnalogueGain, ControlInfo(1.0f, 32.0f) },
		{ &controls::ColourGains, ControlInfo(0.0f, 32.0f) },
		{ &controls::Brightness, ControlInfo(-1.0f, 1.0f) },
	}, controls::controls);

namespace libcamera {

static bool operator==(const ControlInfoMap &lhs, const ControlInfoMap &rhs)
{
	return SerializationTest::equals(lhs, rhs);
}

} /* namespace libcamera */

template<typename T>
int testPodSerdes(T in)
{
	std::vector<uint8_t> buf;
	std::vector<SharedFD> fds;

	std::tie(buf, fds) = IPADataSerializer<T>::serialize(in);
	T out = IPADataSerializer<T>::deserialize(buf, fds);
	if (in == out)
		return TestPass;

	char *name = abi::__cxa_demangle(typeid(T).name(), nullptr,
					 nullptr, nullptr);
	cerr << "Deserialized " << name << " doesn't match original" << endl;
	free(name);
	return TestFail;
}

template<typename T>
int testVectorSerdes(const std::vector<T> &in,
		     ControlSerializer *cs = nullptr)
{
	std::vector<uint8_t> buf;
	std::vector<SharedFD> fds;

	std::tie(buf, fds) = IPADataSerializer<std::vector<T>>::serialize(in, cs);
	std::vector<T> out = IPADataSerializer<std::vector<T>>::deserialize(buf, fds, cs);
	if (in == out)
		return TestPass;

	char *name = abi::__cxa_demangle(typeid(T).name(), nullptr,
					 nullptr, nullptr);
	cerr << "Deserialized std::vector<" << name
	     << "> doesn't match original" << endl;
	free(name);
	return TestFail;
}

template<typename K, typename V>
int testMapSerdes(const std::map<K, V> &in,
		  ControlSerializer *cs = nullptr)
{
	std::vector<uint8_t> buf;
	std::vector<SharedFD> fds;

	std::tie(buf, fds) = IPADataSerializer<std::map<K, V>>::serialize(in, cs);
	std::map<K, V> out = IPADataSerializer<std::map<K, V>>::deserialize(buf, fds, cs);
	if (in == out)
		return TestPass;

	char *nameK = abi::__cxa_demangle(typeid(K).name(), nullptr,
					  nullptr, nullptr);
	char *nameV = abi::__cxa_demangle(typeid(V).name(), nullptr,
					  nullptr, nullptr);
	cerr << "Deserialized std::map<" << nameK << ", " << nameV
	     << "> doesn't match original" << endl;
	free(nameK);
	free(nameV);
	return TestFail;
}

class IPADataSerializerTest : public CameraTest, public Test
{
public:
	IPADataSerializerTest()
		: CameraTest("platform/vimc.0 Sensor B")
	{
	}

protected:
	int init() override
	{
		return status_;
	}

	int run() override
	{
		int ret;

		ret = testControls();
		if (ret != TestPass)
			return ret;

		ret = testVector();
		if (ret != TestPass)
			return ret;

		ret = testMap();
		if (ret != TestPass)
			return ret;

		ret = testPod();
		if (ret != TestPass)
			return ret;

		return TestPass;
	}

private:
	ControlList generateControlList(const ControlInfoMap &infoMap)
	{
		/* Create a control list with three controls. */
		ControlList list(infoMap);

		list.set(controls::Brightness, 0.5f);
		list.set(controls::Contrast, 1.2f);
		list.set(controls::Saturation, 0.2f);

		return list;
	}

	int testControls()
	{
		ControlSerializer cs(ControlSerializer::Role::Proxy);

		const ControlInfoMap &infoMap = camera_->controls();
		ControlList list = generateControlList(infoMap);

		std::vector<uint8_t> infoMapBuf;
		std::tie(infoMapBuf, std::ignore) =
			IPADataSerializer<ControlInfoMap>::serialize(infoMap, &cs);

		std::vector<uint8_t> listBuf;
		std::tie(listBuf, std::ignore) =
			IPADataSerializer<ControlList>::serialize(list, &cs);

		const ControlInfoMap infoMapOut =
			IPADataSerializer<ControlInfoMap>::deserialize(infoMapBuf, &cs);

		ControlList listOut = IPADataSerializer<ControlList>::deserialize(listBuf, &cs);

		if (!SerializationTest::equals(infoMap, infoMapOut)) {
			cerr << "Deserialized map doesn't match original" << endl;
			return TestFail;
		}

		if (!SerializationTest::equals(list, listOut)) {
			cerr << "Deserialized list doesn't match original" << endl;
			return TestFail;
		}

		return TestPass;
	}

	int testVector()
	{
		ControlSerializer cs(ControlSerializer::Role::Proxy);

		/*
		 * We don't test SharedFD serdes because it dup()s, so we
		 * can't check for equality.
		 */
		std::vector<uint8_t>  vecUint8  = { 1, 2, 3, 4, 5, 6 };
		std::vector<uint16_t> vecUint16 = { 1, 2, 3, 4, 5, 6 };
		std::vector<uint32_t> vecUint32 = { 1, 2, 3, 4, 5, 6 };
		std::vector<uint64_t> vecUint64 = { 1, 2, 3, 4, 5, 6 };
		std::vector<int8_t>   vecInt8   = { 1, 2, 3, -4, 5, -6 };
		std::vector<int16_t>  vecInt16  = { 1, 2, 3, -4, 5, -6 };
		std::vector<int32_t>  vecInt32  = { 1, 2, 3, -4, 5, -6 };
		std::vector<int64_t>  vecInt64  = { 1, 2, 3, -4, 5, -6 };
		std::vector<float>    vecFloat  = { 1.1, 2.2, 3.3, -4.4, 5.5, -6.6 };
		std::vector<double>   vecDouble = { 1.1, 2.2, 3.3, -4.4, 5.5, -6.6 };
		std::vector<bool>     vecBool   = { true, true, false, false, true, false };
		std::vector<std::string>   vecString = { "foo", "bar", "baz" };
		std::vector<ControlInfoMap> vecControlInfoMap = {
			camera_->controls(),
			Controls,
		};

		std::vector<uint8_t> buf;
		std::vector<SharedFD> fds;

		if (testVectorSerdes(vecUint8) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecUint16) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecUint32) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecUint64) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecInt8) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecInt16) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecInt32) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecInt64) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecFloat) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecDouble) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecBool) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecString) != TestPass)
			return TestFail;

		if (testVectorSerdes(vecControlInfoMap, &cs) != TestPass)
			return TestFail;

		return TestPass;
	}

	int testMap()
	{
		ControlSerializer cs(ControlSerializer::Role::Proxy);

		/*
		 * Realistically, only string and integral keys.
		 * Test simple, complex, and nested compound value.
		 */
		std::map<uint64_t, std::string> mapUintStr =
			{ { 101, "foo" }, { 102, "bar" }, { 103, "baz" } };
		std::map<int64_t, std::string> mapIntStr =
			{ { 101, "foo" }, { -102, "bar" }, { -103, "baz" } };
		std::map<std::string, std::string> mapStrStr =
			{ { "a", "foo" }, { "b", "bar" }, { "c", "baz" } };
		std::map<uint64_t, ControlInfoMap> mapUintCIM =
			{ { 201, camera_->controls() }, { 202, Controls } };
		std::map<int64_t, ControlInfoMap> mapIntCIM =
			{ { 201, camera_->controls() }, { -202, Controls } };
		std::map<std::string, ControlInfoMap> mapStrCIM =
			{ { "a", camera_->controls() }, { "b", Controls } };
		std::map<uint64_t, std::vector<uint8_t>> mapUintBVec =
			{ { 301, { 1, 2, 3 } }, { 302, { 4, 5, 6 } }, { 303, { 7, 8, 9 } } };
		std::map<int64_t, std::vector<uint8_t>> mapIntBVec =
			{ { 301, { 1, 2, 3 } }, { -302, { 4, 5, 6} }, { -303, { 7, 8, 9 } } };
		std::map<std::string, std::vector<uint8_t>> mapStrBVec =
			{ { "a", { 1, 2, 3 } }, { "b", { 4, 5, 6 } }, { "c", { 7, 8, 9 } } };

		std::vector<uint8_t> buf;
		std::vector<SharedFD> fds;

		if (testMapSerdes(mapUintStr) != TestPass)
			return TestFail;

		if (testMapSerdes(mapIntStr) != TestPass)
			return TestFail;

		if (testMapSerdes(mapStrStr) != TestPass)
			return TestFail;

		if (testMapSerdes(mapUintCIM, &cs) != TestPass)
			return TestFail;

		if (testMapSerdes(mapIntCIM,  &cs) != TestPass)
			return TestFail;

		if (testMapSerdes(mapStrCIM,  &cs) != TestPass)
			return TestFail;

		if (testMapSerdes(mapUintBVec) != TestPass)
			return TestFail;

		if (testMapSerdes(mapIntBVec) != TestPass)
			return TestFail;

		if (testMapSerdes(mapStrBVec) != TestPass)
			return TestFail;

		return TestPass;
	}

	int testPod()
	{
		uint32_t u32min = std::numeric_limits<uint32_t>::min();
		uint32_t u32max = std::numeric_limits<uint32_t>::max();
		uint32_t u32one = 1;
		int32_t  i32min = std::numeric_limits<int32_t>::min();
		int32_t  i32max = std::numeric_limits<int32_t>::max();
		int32_t  i32one = 1;

		uint64_t u64min = std::numeric_limits<uint64_t>::min();
		uint64_t u64max = std::numeric_limits<uint64_t>::max();
		uint64_t u64one = 1;
		int64_t  i64min = std::numeric_limits<int64_t>::min();
		int64_t  i64max = std::numeric_limits<int64_t>::max();
		int64_t  i64one = 1;

		float  flow = std::numeric_limits<float>::lowest();
		float  fmin = std::numeric_limits<float>::min();
		float  fmax = std::numeric_limits<float>::max();
		float  falmostOne = 1 + 1.0e-37;
		double dlow = std::numeric_limits<double>::lowest();
		double dmin = std::numeric_limits<double>::min();
		double dmax = std::numeric_limits<double>::max();
		double dalmostOne = 1 + 1.0e-307;

		bool t = true;
		bool f = false;

		std::stringstream ss;
		for (unsigned int i = 0; i < (1 << 11); i++)
			ss << "0123456789";

		std::string strLong = ss.str();
		std::string strEmpty = "";

		std::vector<uint8_t> buf;
		std::vector<SharedFD> fds;

		if (testPodSerdes(u32min) != TestPass)
			return TestFail;

		if (testPodSerdes(u32max) != TestPass)
			return TestFail;

		if (testPodSerdes(u32one) != TestPass)
			return TestFail;

		if (testPodSerdes(i32min) != TestPass)
			return TestFail;

		if (testPodSerdes(i32max) != TestPass)
			return TestFail;

		if (testPodSerdes(i32one) != TestPass)
			return TestFail;

		if (testPodSerdes(u64min) != TestPass)
			return TestFail;

		if (testPodSerdes(u64max) != TestPass)
			return TestFail;

		if (testPodSerdes(u64one) != TestPass)
			return TestFail;

		if (testPodSerdes(i64min) != TestPass)
			return TestFail;

		if (testPodSerdes(i64max) != TestPass)
			return TestFail;

		if (testPodSerdes(i64one) != TestPass)
			return TestFail;

		if (testPodSerdes(flow) != TestPass)
			return TestFail;

		if (testPodSerdes(fmin) != TestPass)
			return TestFail;

		if (testPodSerdes(fmax) != TestPass)
			return TestFail;

		if (testPodSerdes(falmostOne) != TestPass)
			return TestFail;

		if (testPodSerdes(dlow) != TestPass)
			return TestFail;

		if (testPodSerdes(dmin) != TestPass)
			return TestFail;

		if (testPodSerdes(dmax) != TestPass)
			return TestFail;

		if (testPodSerdes(dalmostOne) != TestPass)
			return TestFail;

		if (testPodSerdes(t) != TestPass)
			return TestFail;

		if (testPodSerdes(f) != TestPass)
			return TestFail;

		if (testPodSerdes(strLong) != TestPass)
			return TestFail;

		if (testPodSerdes(strEmpty) != TestPass)
			return TestFail;

		return TestPass;
	}
};

TEST_REGISTER(IPADataSerializerTest)
