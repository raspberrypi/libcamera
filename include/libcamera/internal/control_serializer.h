/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_serializer.h - Control (de)serializer
 */
#ifndef __LIBCAMERA_INTERNAL_CONTROL_SERIALIZER_H__
#define __LIBCAMERA_INTERNAL_CONTROL_SERIALIZER_H__

#include <map>
#include <memory>
#include <vector>

#include <libcamera/controls.h>

namespace libcamera {

class ByteStreamBuffer;

class ControlSerializer
{
public:
	ControlSerializer();

	void reset();

	static size_t binarySize(const ControlInfoMap &infoMap);
	static size_t binarySize(const ControlList &list);

	int serialize(const ControlInfoMap &infoMap, ByteStreamBuffer &buffer);
	int serialize(const ControlList &list, ByteStreamBuffer &buffer);

	template<typename T>
	T deserialize(ByteStreamBuffer &buffer);

private:
	static size_t binarySize(const ControlValue &value);
	static size_t binarySize(const ControlInfo &info);

	static void store(const ControlValue &value, ByteStreamBuffer &buffer);
	static void store(const ControlInfo &info, ByteStreamBuffer &buffer);

	ControlValue loadControlValue(ControlType type, ByteStreamBuffer &buffer,
				      bool isArray = false, unsigned int count = 1);
	ControlInfo loadControlInfo(ControlType type, ByteStreamBuffer &buffer);

	unsigned int serial_;
	std::vector<std::unique_ptr<ControlId>> controlIds_;
	std::map<unsigned int, ControlInfoMap> infoMaps_;
	std::map<const ControlInfoMap *, unsigned int> infoMapHandles_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_INTERNAL_CONTROL_SERIALIZER_H__ */
