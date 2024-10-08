/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Debug metadata helpers
 */

#pragma once

#include <libcamera/control_ids.h>

namespace libcamera {

class DebugMetadata
{
public:
	DebugMetadata() = default;

	void enableByControl(const ControlList &controls);
	void enable(bool enable = true);
	void setParent(DebugMetadata *parent);
	void moveEntries(ControlList &list);

	template<typename T, typename V>
	void set(const Control<T> &ctrl, const V &value)
	{
		if (parent_) {
			parent_->set(ctrl, value);
			return;
		}

		if (!enabled_)
			return;

		cache_.set(ctrl, value);
	}

	void set(unsigned int id, const ControlValue &value);

private:
	bool enabled_ = false;
	DebugMetadata *parent_ = nullptr;
	ControlList cache_;
};

} /* namespace libcamera */
