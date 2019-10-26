/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * control_serializer.h - Control (de)serializer
 */
#ifndef __LIBCAMERA_CONTROL_SERIALIZER_H__
#define __LIBCAMERA_CONTROL_SERIALIZER_H__

#include <map>
#include <memory>
#include <vector>

#include <libcamera/controls.h>

namespace libcamera {

class ByteStreamBuffer;

class ControlSerializer
{
public:
	void reset();

	static size_t binarySize(const ControlInfoMap &info);
	static size_t binarySize(const ControlList &list);

	int serialize(const ControlInfoMap &info, ByteStreamBuffer &buffer);
	int serialize(const ControlList &list, ByteStreamBuffer &buffer);

	template<typename T>
	T deserialize(ByteStreamBuffer &buffer);

private:
	static size_t binarySize(const ControlValue &value);
	static size_t binarySize(const ControlRange &range);

	static void store(const ControlValue &value, ByteStreamBuffer &buffer);
	static void store(const ControlRange &range, ByteStreamBuffer &buffer);

	template<typename T>
	T load(ControlType type, ByteStreamBuffer &b);

	unsigned int serial_;
	std::vector<std::unique_ptr<ControlId>> controlIds_;
	std::map<unsigned int, ControlInfoMap> infoMaps_;
	std::map<const ControlInfoMap *, unsigned int> infoMapHandles_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CONTROL_SERIALIZER_H__ */
