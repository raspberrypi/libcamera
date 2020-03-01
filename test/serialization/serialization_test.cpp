/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * serialization_test.cpp - Base class for serialization tests
 */

#include "serialization_test.h"

#include <algorithm>
#include <iostream>
#include <map>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>

#include "test.h"

using namespace std;
using namespace libcamera;

bool SerializationTest::equals(const ControlInfoMap &lhs, const ControlInfoMap &rhs)
{
	std::map<unsigned int, ControlInfo> rlhs;
	std::transform(lhs.begin(), lhs.end(), std::inserter(rlhs, rlhs.end()),
			[](const ControlInfoMap::value_type &v)
				-> decltype(rlhs)::value_type
			{
				return { v.first->id(), v.second };
			});

	std::map<unsigned int, ControlInfo> rrhs;
	std::transform(rhs.begin(), rhs.end(), std::inserter(rrhs, rrhs.end()),
			[](const ControlInfoMap::value_type &v)
				-> decltype(rrhs)::value_type
			{
				return { v.first->id(), v.second };
			});

	if (rlhs == rrhs)
		return true;

	cerr << "lhs:" << endl;
	for (const auto &value : rlhs)
		cerr << "- " << value.first << ": "
		     << value.second.toString() << endl;

	cerr << "rhs:" << endl;
	for (const auto &value : rrhs)
		cerr << "- " << value.first << ": "
		     << value.second.toString() << endl;

	return false;
}

bool SerializationTest::equals(const ControlList &lhs, const ControlList &rhs)
{
	std::map<unsigned int, ControlValue> rlhs;
	std::transform(lhs.begin(), lhs.end(), std::inserter(rlhs, rlhs.end()),
			[](const std::pair<unsigned int, ControlValue> &v)
				-> decltype(rlhs)::value_type
			{
				return { v.first, v.second };
			});

	std::map<unsigned int, ControlValue> rrhs;
	std::transform(rhs.begin(), rhs.end(), std::inserter(rrhs, rrhs.end()),
			[](const std::pair<unsigned int, ControlValue> &v)
				-> decltype(rrhs)::value_type
			{
				return { v.first, v.second };
			});

	if (rlhs == rrhs)
		return true;

	cerr << "lhs:" << endl;
	for (const auto &value : rlhs)
		cerr << "- " << value.first << ": "
		     << value.second.toString() << endl;

	cerr << "rhs:" << endl;
	for (const auto &value : rrhs)
		cerr << "- " << value.first << ": "
		     << value.second.toString() << endl;

	return false;
}
