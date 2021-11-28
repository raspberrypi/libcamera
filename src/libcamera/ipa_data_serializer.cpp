/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipa_data_serializer.cpp - Image Processing Algorithm data serializer
 */

#include "libcamera/internal/ipa_data_serializer.h"

#include <unistd.h>

#include <libcamera/base/log.h>

/**
 * \file ipa_data_serializer.h
 * \brief IPA Data Serializer
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(IPADataSerializer)

/**
 * \class IPADataSerializer
 * \brief IPA Data Serializer
 *
 * Static template class that provides functions for serializing and
 * deserializing IPA data.
 *
 * \todo Switch to Span instead of byte and fd vector
 *
 * \todo Harden the vector and map deserializer
 *
 * \todo For SharedFDs, instead of storing a validity flag, store an
 * index into the fd array. This will allow us to use views instead of copying.
 */

namespace {

/**
 * \fn template<typename T> void appendPOD(std::vector<uint8_t> &vec, T val)
 * \brief Append POD to end of byte vector, in little-endian order
 * \tparam T Type of POD to append
 * \param[in] vec Byte vector to append to
 * \param[in] val Value to append
 *
 * This function is meant to be used by the IPA data serializer, and the
 * generated IPA proxies.
 */

/**
 * \fn template<typename T> T readPOD(std::vector<uint8_t>::iterator it, size_t pos,
 * 				      std::vector<uint8_t>::iterator end)
 * \brief Read POD from byte vector, in little-endian order
 * \tparam T Type of POD to read
 * \param[in] it Iterator of byte vector to read from
 * \param[in] pos Index in byte vector to read from
 * \param[in] end Iterator marking end of byte vector
 *
 * This function is meant to be used by the IPA data serializer, and the
 * generated IPA proxies.
 *
 * If the \a pos plus the byte-width of the desired POD is past \a end, it is
 * a fata error will occur, as it means there is insufficient data for
 * deserialization, which should never happen.
 *
 * \return The POD read from \a it at index \a pos
 */

/**
 * \fn template<typename T> T readPOD(std::vector<uint8_t> &vec, size_t pos)
 * \brief Read POD from byte vector, in little-endian order
 * \tparam T Type of POD to read
 * \param[in] vec Byte vector to read from
 * \param[in] pos Index in vec to start reading from
 *
 * This function is meant to be used by the IPA data serializer, and the
 * generated IPA proxies.
 *
 * If the \a pos plus the byte-width of the desired POD is past the end of
 * \a vec, a fatal error will occur, as it means there is insufficient data
 * for deserialization, which should never happen.
 *
 * \return The POD read from \a vec at index \a pos
 */

} /* namespace */

/**
 * \fn template<typename T> IPADataSerializer<T>::serialize(
 * 	T data,
 * 	ControlSerializer *cs = nullptr)
 * \brief Serialize an object into byte vector and fd vector
 * \tparam T Type of object to serialize
 * \param[in] data Object to serialize
 * \param[in] cs ControlSerializer
 *
 * \a cs is only necessary if the object type \a T or its members contain
 * ControlList or ControlInfoMap.
 *
 * \return Tuple of byte vector and fd vector, that is the serialized form
 * of \a data
 */

/**
 * \fn template<typename T> IPADataSerializer<T>::deserialize(
 * 	const std::vector<uint8_t> &data,
 * 	ControlSerializer *cs = nullptr)
 * \brief Deserialize byte vector into an object
 * \tparam T Type of object to deserialize to
 * \param[in] data Byte vector to deserialize from
 * \param[in] cs ControlSerializer
 *
 * This version of deserialize() can be used if the object type \a T and its
 * members don't have any SharedFD.
 *
 * \a cs is only necessary if the object type \a T or its members contain
 * ControlList or ControlInfoMap.
 *
 * \return The deserialized object
 */

/**
 * \fn template<typename T> IPADataSerializer<T>::deserialize(
 * 	std::vector<uint8_t>::const_iterator dataBegin,
 * 	std::vector<uint8_t>::const_iterator dataEnd,
 * 	ControlSerializer *cs = nullptr)
 * \brief Deserialize byte vector into an object
 * \tparam T Type of object to deserialize to
 * \param[in] dataBegin Begin iterator of byte vector to deserialize from
 * \param[in] dataEnd End iterator of byte vector to deserialize from
 * \param[in] cs ControlSerializer
 *
 * This version of deserialize() can be used if the object type \a T and its
 * members don't have any SharedFD.
 *
 * \a cs is only necessary if the object type \a T or its members contain
 * ControlList or ControlInfoMap.
 *
 * \return The deserialized object
 */

/**
 * \fn template<typename T> IPADataSerializer<T>::deserialize(
 * 	const std::vector<uint8_t> &data,
 * 	const std::vector<SharedFD> &fds,
 * 	ControlSerializer *cs = nullptr)
 * \brief Deserialize byte vector and fd vector into an object
 * \tparam T Type of object to deserialize to
 * \param[in] data Byte vector to deserialize from
 * \param[in] fds Fd vector to deserialize from
 * \param[in] cs ControlSerializer
 *
 * This version of deserialize() (or the iterator version) must be used if
 * the object type \a T or its members contain SharedFD.
 *
 * \a cs is only necessary if the object type \a T or its members contain
 * ControlList or ControlInfoMap.
 *
 * \return The deserialized object
 */

/**
 * \fn template<typename T> IPADataSerializer::deserialize(
 * 	std::vector<uint8_t>::const_iterator dataBegin,
 * 	std::vector<uint8_t>::const_iterator dataEnd,
 * 	std::vector<SharedFD>::const_iterator fdsBegin,
 * 	std::vector<SharedFD>::const_iterator fdsEnd,
 * 	ControlSerializer *cs = nullptr)
 * \brief Deserialize byte vector and fd vector into an object
 * \tparam T Type of object to deserialize to
 * \param[in] dataBegin Begin iterator of byte vector to deserialize from
 * \param[in] dataEnd End iterator of byte vector to deserialize from
 * \param[in] fdsBegin Begin iterator of fd vector to deserialize from
 * \param[in] fdsEnd End iterator of fd vector to deserialize from
 * \param[in] cs ControlSerializer
 *
 * This version of deserialize() (or the vector version) must be used if
 * the object type \a T or its members contain SharedFD.
 *
 * \a cs is only necessary if the object type \a T or its members contain
 * ControlList or ControlInfoMap.
 *
 * \return The deserialized object
 */

#ifndef __DOXYGEN__

#define DEFINE_POD_SERIALIZER(type)					\
									\
template<>								\
std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>		\
IPADataSerializer<type>::serialize(const type &data,			\
				  [[maybe_unused]] ControlSerializer *cs) \
{									\
	std::vector<uint8_t> dataVec;					\
	dataVec.reserve(sizeof(type));					\
	appendPOD<type>(dataVec, data);					\
									\
	return { dataVec, {} };						\
}									\
									\
template<>								\
type IPADataSerializer<type>::deserialize(std::vector<uint8_t>::const_iterator dataBegin, \
					  std::vector<uint8_t>::const_iterator dataEnd, \
					  [[maybe_unused]] ControlSerializer *cs) \
{									\
	return readPOD<type>(dataBegin, 0, dataEnd);			\
}									\
									\
template<>								\
type IPADataSerializer<type>::deserialize(const std::vector<uint8_t> &data, \
					  ControlSerializer *cs)	\
{									\
	return deserialize(data.cbegin(), data.end(), cs);		\
}									\
									\
template<>								\
type IPADataSerializer<type>::deserialize(const std::vector<uint8_t> &data, \
					  [[maybe_unused]] const std::vector<SharedFD> &fds, \
					  ControlSerializer *cs)	\
{									\
	return deserialize(data.cbegin(), data.end(), cs);		\
}									\
									\
template<>								\
type IPADataSerializer<type>::deserialize(std::vector<uint8_t>::const_iterator dataBegin, \
					  std::vector<uint8_t>::const_iterator dataEnd, \
					  [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin, \
					  [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd, \
					  ControlSerializer *cs)	\
{									\
	return deserialize(dataBegin, dataEnd, cs);			\
}

DEFINE_POD_SERIALIZER(bool)
DEFINE_POD_SERIALIZER(uint8_t)
DEFINE_POD_SERIALIZER(uint16_t)
DEFINE_POD_SERIALIZER(uint32_t)
DEFINE_POD_SERIALIZER(uint64_t)
DEFINE_POD_SERIALIZER(int8_t)
DEFINE_POD_SERIALIZER(int16_t)
DEFINE_POD_SERIALIZER(int32_t)
DEFINE_POD_SERIALIZER(int64_t)
DEFINE_POD_SERIALIZER(float)
DEFINE_POD_SERIALIZER(double)

/*
 * Strings are serialized simply by converting by {string.cbegin(), string.end()}.
 * The size of the string is recorded by the container (struct, vector, map, or
 * function parameter serdes).
 */
template<>
std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
IPADataSerializer<std::string>::serialize(const std::string &data,
					  [[maybe_unused]] ControlSerializer *cs)
{
	return { { data.cbegin(), data.end() }, {} };
}

template<>
std::string
IPADataSerializer<std::string>::deserialize(const std::vector<uint8_t> &data,
					    [[maybe_unused]] ControlSerializer *cs)
{
	return { data.cbegin(), data.cend() };
}

template<>
std::string
IPADataSerializer<std::string>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					    std::vector<uint8_t>::const_iterator dataEnd,
					    [[maybe_unused]] ControlSerializer *cs)
{
	return { dataBegin, dataEnd };
}

template<>
std::string
IPADataSerializer<std::string>::deserialize(const std::vector<uint8_t> &data,
					    [[maybe_unused]] const std::vector<SharedFD> &fds,
					    [[maybe_unused]] ControlSerializer *cs)
{
	return { data.cbegin(), data.cend() };
}

template<>
std::string
IPADataSerializer<std::string>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					    std::vector<uint8_t>::const_iterator dataEnd,
					    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
					    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
					    [[maybe_unused]] ControlSerializer *cs)
{
	return { dataBegin, dataEnd };
}

/*
 * ControlList is serialized as:
 *
 * 4 bytes - uint32_t Size of serialized ControlInfoMap, in bytes
 * 4 bytes - uint32_t Size of serialized ControlList, in bytes
 * X bytes - Serialized ControlInfoMap (using ControlSerializer)
 * X bytes - Serialized ControlList (using ControlSerializer)
 *
 * If data.infoMap() is nullptr, then the default controls::controls will
 * be used. The serialized ControlInfoMap will have zero length.
 */
template<>
std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
IPADataSerializer<ControlList>::serialize(const ControlList &data, ControlSerializer *cs)
{
	if (!cs)
		LOG(IPADataSerializer, Fatal)
			<< "ControlSerializer not provided for serialization of ControlList";

	size_t size;
	std::vector<uint8_t> infoData;
	int ret;

	/*
	 * \todo Revisit this opportunistic serialization of the
	 * ControlInfoMap, as it could be fragile
	 */
	if (data.infoMap() && !cs->isCached(*data.infoMap())) {
		size = cs->binarySize(*data.infoMap());
		infoData.resize(size);
		ByteStreamBuffer buffer(infoData.data(), infoData.size());
		ret = cs->serialize(*data.infoMap(), buffer);

		if (ret < 0 || buffer.overflow()) {
			LOG(IPADataSerializer, Error) << "Failed to serialize ControlList's ControlInfoMap";
			return { {}, {} };
		}
	}

	size = cs->binarySize(data);
	std::vector<uint8_t> listData(size);
	ByteStreamBuffer buffer(listData.data(), listData.size());
	ret = cs->serialize(data, buffer);

	if (ret < 0 || buffer.overflow()) {
		LOG(IPADataSerializer, Error) << "Failed to serialize ControlList";
		return { {}, {} };
	}

	std::vector<uint8_t> dataVec;
	dataVec.reserve(8 + infoData.size() + listData.size());
	appendPOD<uint32_t>(dataVec, infoData.size());
	appendPOD<uint32_t>(dataVec, listData.size());
	dataVec.insert(dataVec.end(), infoData.begin(), infoData.end());
	dataVec.insert(dataVec.end(), listData.begin(), listData.end());

	return { dataVec, {} };
}

template<>
ControlList
IPADataSerializer<ControlList>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					    std::vector<uint8_t>::const_iterator dataEnd,
					    ControlSerializer *cs)
{
	if (!cs)
		LOG(IPADataSerializer, Fatal)
			<< "ControlSerializer not provided for deserialization of ControlList";

	if (std::distance(dataBegin, dataEnd) < 8)
		return {};

	uint32_t infoDataSize = readPOD<uint32_t>(dataBegin, 0, dataEnd);
	uint32_t listDataSize = readPOD<uint32_t>(dataBegin, 4, dataEnd);

	std::vector<uint8_t>::const_iterator it = dataBegin + 8;

	if (infoDataSize + listDataSize < infoDataSize ||
	    static_cast<uint32_t>(std::distance(it, dataEnd)) < infoDataSize + listDataSize)
		return {};

	if (infoDataSize > 0) {
		ByteStreamBuffer buffer(&*it, infoDataSize);
		ControlInfoMap map = cs->deserialize<ControlInfoMap>(buffer);
		/* It's fine if map is empty. */
		if (buffer.overflow()) {
			LOG(IPADataSerializer, Error)
				<< "Failed to deserialize ControlLists's ControlInfoMap: buffer overflow";
			return ControlList();
		}
	}

	it += infoDataSize;
	ByteStreamBuffer buffer(&*it, listDataSize);
	ControlList list = cs->deserialize<ControlList>(buffer);
	if (buffer.overflow())
		LOG(IPADataSerializer, Error) << "Failed to deserialize ControlList: buffer overflow";

	return list;
}

template<>
ControlList
IPADataSerializer<ControlList>::deserialize(const std::vector<uint8_t> &data,
					    ControlSerializer *cs)
{
	return deserialize(data.cbegin(), data.end(), cs);
}

template<>
ControlList
IPADataSerializer<ControlList>::deserialize(const std::vector<uint8_t> &data,
					    [[maybe_unused]] const std::vector<SharedFD> &fds,
					    ControlSerializer *cs)
{
	return deserialize(data.cbegin(), data.end(), cs);
}

template<>
ControlList
IPADataSerializer<ControlList>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					    std::vector<uint8_t>::const_iterator dataEnd,
					    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
					    [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
					    ControlSerializer *cs)
{
	return deserialize(dataBegin, dataEnd, cs);
}

/*
 * const ControlInfoMap is serialized as:
 *
 * 4 bytes - uint32_t Size of serialized ControlInfoMap, in bytes
 * X bytes - Serialized ControlInfoMap (using ControlSerializer)
 */
template<>
std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
IPADataSerializer<ControlInfoMap>::serialize(const ControlInfoMap &map,
					     ControlSerializer *cs)
{
	if (!cs)
		LOG(IPADataSerializer, Fatal)
			<< "ControlSerializer not provided for serialization of ControlInfoMap";

	size_t size = cs->binarySize(map);
	std::vector<uint8_t> infoData(size);
	ByteStreamBuffer buffer(infoData.data(), infoData.size());
	int ret = cs->serialize(map, buffer);

	if (ret < 0 || buffer.overflow()) {
		LOG(IPADataSerializer, Error) << "Failed to serialize ControlInfoMap";
		return { {}, {} };
	}

	std::vector<uint8_t> dataVec;
	appendPOD<uint32_t>(dataVec, infoData.size());
	dataVec.insert(dataVec.end(), infoData.begin(), infoData.end());

	return { dataVec, {} };
}

template<>
ControlInfoMap
IPADataSerializer<ControlInfoMap>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					       std::vector<uint8_t>::const_iterator dataEnd,
					       ControlSerializer *cs)
{
	if (!cs)
		LOG(IPADataSerializer, Fatal)
			<< "ControlSerializer not provided for deserialization of ControlInfoMap";

	if (std::distance(dataBegin, dataEnd) < 4)
		return {};

	uint32_t infoDataSize = readPOD<uint32_t>(dataBegin, 0, dataEnd);

	std::vector<uint8_t>::const_iterator it = dataBegin + 4;

	if (static_cast<uint32_t>(std::distance(it, dataEnd)) < infoDataSize)
		return {};

	ByteStreamBuffer buffer(&*it, infoDataSize);
	ControlInfoMap map = cs->deserialize<ControlInfoMap>(buffer);

	return map;
}

template<>
ControlInfoMap
IPADataSerializer<ControlInfoMap>::deserialize(const std::vector<uint8_t> &data,
					       ControlSerializer *cs)
{
	return deserialize(data.cbegin(), data.end(), cs);
}

template<>
ControlInfoMap
IPADataSerializer<ControlInfoMap>::deserialize(const std::vector<uint8_t> &data,
					       [[maybe_unused]] const std::vector<SharedFD> &fds,
					       ControlSerializer *cs)
{
	return deserialize(data.cbegin(), data.end(), cs);
}

template<>
ControlInfoMap
IPADataSerializer<ControlInfoMap>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
					       std::vector<uint8_t>::const_iterator dataEnd,
					       [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsBegin,
					       [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
					       ControlSerializer *cs)
{
	return deserialize(dataBegin, dataEnd, cs);
}

/*
 * SharedFD instances are serialized into four bytes that tells if the SharedFD
 * is valid or not. If it is valid, then for serialization the fd will be
 * written to the fd vector, or for deserialization the fd vector const_iterator
 * will be valid.
 *
 * This validity is necessary so that we don't send -1 fd over sendmsg(). It
 * also allows us to simply send the entire fd vector into the deserializer
 * and it will be recursively consumed as necessary.
 */
template<>
std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
IPADataSerializer<SharedFD>::serialize(const SharedFD &data,
				       [[maybe_unused]] ControlSerializer *cs)
{
	std::vector<uint8_t> dataVec;
	std::vector<SharedFD> fdVec;

	/*
	 * Store as uint32_t to prepare for conversion from validity flag
	 * to index, and for alignment.
	 */
	appendPOD<uint32_t>(dataVec, data.isValid());

	if (data.isValid())
		fdVec.push_back(data);


	return { dataVec, fdVec };
}

template<>
SharedFD IPADataSerializer<SharedFD>::deserialize([[maybe_unused]] std::vector<uint8_t>::const_iterator dataBegin,
						  [[maybe_unused]] std::vector<uint8_t>::const_iterator dataEnd,
						  std::vector<SharedFD>::const_iterator fdsBegin,
						  std::vector<SharedFD>::const_iterator fdsEnd,
						  [[maybe_unused]] ControlSerializer *cs)
{
	ASSERT(std::distance(dataBegin, dataEnd) >= 4);

	uint32_t valid = readPOD<uint32_t>(dataBegin, 0, dataEnd);

	ASSERT(!(valid && std::distance(fdsBegin, fdsEnd) < 1));

	return valid ? *fdsBegin : SharedFD();
}

template<>
SharedFD IPADataSerializer<SharedFD>::deserialize(const std::vector<uint8_t> &data,
						  const std::vector<SharedFD> &fds,
						  [[maybe_unused]] ControlSerializer *cs)
{
	return deserialize(data.cbegin(), data.end(), fds.cbegin(), fds.end());
}

/*
 * FrameBuffer::Plane is serialized as:
 *
 * 4 byte  - SharedFD
 * 4 bytes - uint32_t Offset
 * 4 bytes - uint32_t Length
 */
template<>
std::tuple<std::vector<uint8_t>, std::vector<SharedFD>>
IPADataSerializer<FrameBuffer::Plane>::serialize(const FrameBuffer::Plane &data,
						 [[maybe_unused]] ControlSerializer *cs)
{
	std::vector<uint8_t> dataVec;
	std::vector<SharedFD> fdsVec;

	std::vector<uint8_t> fdBuf;
	std::vector<SharedFD> fdFds;
	std::tie(fdBuf, fdFds) =
		IPADataSerializer<SharedFD>::serialize(data.fd);
	dataVec.insert(dataVec.end(), fdBuf.begin(), fdBuf.end());
	fdsVec.insert(fdsVec.end(), fdFds.begin(), fdFds.end());

	appendPOD<uint32_t>(dataVec, data.offset);
	appendPOD<uint32_t>(dataVec, data.length);

	return { dataVec, fdsVec };
}

template<>
FrameBuffer::Plane
IPADataSerializer<FrameBuffer::Plane>::deserialize(std::vector<uint8_t>::const_iterator dataBegin,
						   std::vector<uint8_t>::const_iterator dataEnd,
						   std::vector<SharedFD>::const_iterator fdsBegin,
						   [[maybe_unused]] std::vector<SharedFD>::const_iterator fdsEnd,
						   [[maybe_unused]] ControlSerializer *cs)
{
	FrameBuffer::Plane ret;

	ret.fd = IPADataSerializer<SharedFD>::deserialize(dataBegin, dataBegin + 4,
								fdsBegin, fdsBegin + 1);
	ret.offset = readPOD<uint32_t>(dataBegin, 4, dataEnd);
	ret.length = readPOD<uint32_t>(dataBegin, 8, dataEnd);

	return ret;
}

template<>
FrameBuffer::Plane
IPADataSerializer<FrameBuffer::Plane>::deserialize(const std::vector<uint8_t> &data,
						   const std::vector<SharedFD> &fds,
						   ControlSerializer *cs)
{
	return deserialize(data.cbegin(), data.end(), fds.cbegin(), fds.end(), cs);
}

#endif /* __DOXYGEN__ */

} /* namespace libcamera */
