/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipa_data_serializer.h - Image Processing Algorithm data serializer
 */

#pragma once

#include <deque>
#include <iostream>
#include <string.h>
#include <tuple>
#include <type_traits>
#include <vector>

#include <libcamera/base/flags.h>
#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/ipa/ipa_interface.h>

#include "libcamera/internal/byte_stream_buffer.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/control_serializer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPADataSerializer)

namespace {

template<typename T,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
void appendPOD(std::vector<uint8_t> &vec, T val)
{
	constexpr size_t byteWidth = sizeof(val);
	vec.resize(vec.size() + byteWidth);
	memcpy(&*(vec.end() - byteWidth), &val, byteWidth);
}

template<typename T,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
T readPOD(std::vector<uint8_t>::const_iterator it, size_t pos,
	  std::vector<uint8_t>::const_iterator end)
{
	ASSERT(pos + it < end);

	T ret = 0;
	memcpy(&ret, &(*(it + pos)), sizeof(ret));

	return ret;
}

template<typename T,
	 std::enable_if_t<std::is_arithmetic_v<T>> * = nullptr>
T readPOD(std::vector<uint8_t> &vec, size_t pos)
{
	return readPOD<T>(vec.cbegin(), pos, vec.end());
}

} /* namespace */

template<typename T>
class IPADataSerializer
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const T &data, ControlSerializer *cs = nullptr);

	static T deserialize(const std::vector<uint8_t> &data,
			     ControlSerializer *cs = nullptr);
	static T deserialize(std::vector<uint8_t>::const_iterator dataBegin,
			     std::vector<uint8_t>::const_iterator dataEnd,
			     ControlSerializer *cs = nullptr);

	static T deserialize(const std::vector<uint8_t> &data,
			     const std::vector<SharedFD> &fds,
			     ControlSerializer *cs = nullptr);
	static T deserialize(std::vector<uint8_t>::const_iterator dataBegin,
			     std::vector<uint8_t>::const_iterator dataEnd,
			     std::vector<SharedFD>::const_iterator fdsBegin,
			     std::vector<SharedFD>::const_iterator fdsEnd,
			     ControlSerializer *cs = nullptr);
};

#ifndef __DOXYGEN__

/*
 * Serialization format for vector of type V:
 *
 * 4 bytes - uint32_t Length of vector, in number of elements
 *
 * For every element in the vector:
 *
 * 4 bytes - uint32_t Size of element, in bytes
 * 4 bytes - uint32_t Number of fds for the element
 * X bytes - Serialized element
 *
 * \todo Support elements that are references
 */
template<typename V>
class IPADataSerializer<std::vector<V>>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const std::vector<V> &data, ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> dataVec;
		std::vector<SharedFD> fdsVec;

		/* Serialize the length. */
		uint32_t vecLen = data.size();
		appendPOD<uint32_t>(dataVec, vecLen);

		/* Serialize the members. */
		for (auto const &it : data) {
			std::vector<uint8_t> dvec;
			std::vector<SharedFD> fvec;

			std::tie(dvec, fvec) =
				IPADataSerializer<V>::serialize(it, cs);

			appendPOD<uint32_t>(dataVec, dvec.size());
			appendPOD<uint32_t>(dataVec, fvec.size());

			dataVec.insert(dataVec.end(), dvec.begin(), dvec.end());
			fdsVec.insert(fdsVec.end(), fvec.begin(), fvec.end());
		}

		return { dataVec, fdsVec };
	}

	static std::vector<V> deserialize(std::vector<uint8_t> &data, ControlSerializer *cs = nullptr)
	{
		return deserialize(data.cbegin(), data.cend(), cs);
	}

	static std::vector<V> deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					  std::vector<uint8_t>::const_iterator dataEnd,
					  ControlSerializer *cs = nullptr)
	{
		std::vector<SharedFD> fds;
		return deserialize(dataBegin, dataEnd, fds.cbegin(), fds.cend(), cs);
	}

	static std::vector<V> deserialize(std::vector<uint8_t> &data, std::vector<SharedFD> &fds,
					  ControlSerializer *cs = nullptr)
	{
		return deserialize(data.cbegin(), data.cend(), fds.cbegin(), fds.cend(), cs);
	}

	static std::vector<V> deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					  std::vector<uint8_t>::const_iterator dataEnd,
					  std::vector<SharedFD>::const_iterator fdsBegin,
					  [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
					  ControlSerializer *cs = nullptr)
	{
		uint32_t vecLen = readPOD<uint32_t>(dataBegin, 0, dataEnd);
		std::vector<V> ret(vecLen);

		std::vector<uint8_t>::const_iterator dataIter = dataBegin + 4;
		std::vector<SharedFD>::const_iterator fdIter = fdsBegin;
		for (uint32_t i = 0; i < vecLen; i++) {
			uint32_t sizeofData = readPOD<uint32_t>(dataIter, 0, dataEnd);
			uint32_t sizeofFds  = readPOD<uint32_t>(dataIter, 4, dataEnd);
			dataIter += 8;

			ret[i] = IPADataSerializer<V>::deserialize(dataIter,
								   dataIter + sizeofData,
								   fdIter,
								   fdIter + sizeofFds,
								   cs);

			dataIter += sizeofData;
			fdIter += sizeofFds;
		}

		return ret;
	}
};

/*
 * Serialization format for map of key type K and value type V:
 *
 * 4 bytes - uint32_t Length of map, in number of pairs
 *
 * For every pair in the map:
 *
 * 4 bytes - uint32_t Size of key, in bytes
 * 4 bytes - uint32_t Number of fds for the key
 * X bytes - Serialized key
 * 4 bytes - uint32_t Size of value, in bytes
 * 4 bytes - uint32_t Number of fds for the value
 * X bytes - Serialized value
 *
 * \todo Support keys or values that are references
 */
template<typename K, typename V>
class IPADataSerializer<std::map<K, V>>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const std::map<K, V> &data, ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> dataVec;
		std::vector<SharedFD> fdsVec;

		/* Serialize the length. */
		uint32_t mapLen = data.size();
		appendPOD<uint32_t>(dataVec, mapLen);

		/* Serialize the members. */
		for (auto const &it : data) {
			std::vector<uint8_t> dvec;
			std::vector<SharedFD> fvec;

			std::tie(dvec, fvec) =
				IPADataSerializer<K>::serialize(it.first, cs);

			appendPOD<uint32_t>(dataVec, dvec.size());
			appendPOD<uint32_t>(dataVec, fvec.size());

			dataVec.insert(dataVec.end(), dvec.begin(), dvec.end());
			fdsVec.insert(fdsVec.end(), fvec.begin(), fvec.end());

			std::tie(dvec, fvec) =
				IPADataSerializer<V>::serialize(it.second, cs);

			appendPOD<uint32_t>(dataVec, dvec.size());
			appendPOD<uint32_t>(dataVec, fvec.size());

			dataVec.insert(dataVec.end(), dvec.begin(), dvec.end());
			fdsVec.insert(fdsVec.end(), fvec.begin(), fvec.end());
		}

		return { dataVec, fdsVec };
	}

	static std::map<K, V> deserialize(std::vector<uint8_t> &data, ControlSerializer *cs = nullptr)
	{
		return deserialize(data.cbegin(), data.cend(), cs);
	}

	static std::map<K, V> deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					  std::vector<uint8_t>::const_iterator dataEnd,
					  ControlSerializer *cs = nullptr)
	{
		std::vector<SharedFD> fds;
		return deserialize(dataBegin, dataEnd, fds.cbegin(), fds.cend(), cs);
	}

	static std::map<K, V> deserialize(std::vector<uint8_t> &data, std::vector<SharedFD> &fds,
					  ControlSerializer *cs = nullptr)
	{
		return deserialize(data.cbegin(), data.cend(), fds.cbegin(), fds.cend(), cs);
	}

	static std::map<K, V> deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					  std::vector<uint8_t>::const_iterator dataEnd,
					  std::vector<SharedFD>::const_iterator fdsBegin,
					  [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
					  ControlSerializer *cs = nullptr)
	{
		std::map<K, V> ret;

		uint32_t mapLen = readPOD<uint32_t>(dataBegin, 0, dataEnd);

		std::vector<uint8_t>::const_iterator dataIter = dataBegin + 4;
		std::vector<SharedFD>::const_iterator fdIter = fdsBegin;
		for (uint32_t i = 0; i < mapLen; i++) {
			uint32_t sizeofData = readPOD<uint32_t>(dataIter, 0, dataEnd);
			uint32_t sizeofFds  = readPOD<uint32_t>(dataIter, 4, dataEnd);
			dataIter += 8;

			K key = IPADataSerializer<K>::deserialize(dataIter,
								  dataIter + sizeofData,
								  fdIter,
								  fdIter + sizeofFds,
								  cs);

			dataIter += sizeofData;
			fdIter += sizeofFds;
			sizeofData = readPOD<uint32_t>(dataIter, 0, dataEnd);
			sizeofFds  = readPOD<uint32_t>(dataIter, 4, dataEnd);
			dataIter += 8;

			const V value = IPADataSerializer<V>::deserialize(dataIter,
									  dataIter + sizeofData,
									  fdIter,
									  fdIter + sizeofFds,
									  cs);
			ret.insert({ key, value });

			dataIter += sizeofData;
			fdIter += sizeofFds;
		}

		return ret;
	}
};

/* Serialization format for Flags is same as for PODs */
template<typename E>
class IPADataSerializer<Flags<E>>
{
public:
	static std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
	serialize(const Flags<E> &data, [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		std::vector<uint8_t> dataVec;
		dataVec.reserve(sizeof(Flags<E>));
		appendPOD<uint32_t>(dataVec, static_cast<typename Flags<E>::Type>(data));

		return { dataVec, {} };
	}

	static Flags<E> deserialize(std::vector<uint8_t> &data,
				    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		return deserialize(data.cbegin(), data.cend());
	}

	static Flags<E> deserialize(std::vector<uint8_t>::const_iterator dataBegin,
				    std::vector<uint8_t>::const_iterator dataEnd,
				    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		return Flags<E>{ static_cast<E>(readPOD<uint32_t>(dataBegin, 0, dataEnd)) };
	}

	static Flags<E> deserialize(std::vector<uint8_t> &data,
				    [[maybe_unused]] std::vector<SharedFD> &fds,
				    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		return deserialize(data.cbegin(), data.cend());
	}

	static Flags<E> deserialize(std::vector<uint8_t>::const_iterator dataBegin,
				    std::vector<uint8_t>::const_iterator dataEnd,
				    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
				    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
				    [[maybe_unused]] ControlSerializer *cs = nullptr)
	{
		return deserialize(dataBegin, dataEnd);
	}
};

#endif /* __DOXYGEN__ */

} /* namespace libcamera */
