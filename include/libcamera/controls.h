/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.h - Control handling
 */

#ifndef __LIBCAMERA_CONTROLS_H__
#define __LIBCAMERA_CONTROLS_H__

#include <stdint.h>
#include <string>
#include <unordered_map>

#include <libcamera/control_ids.h>

namespace libcamera {

class Camera;

enum ControlValueType {
	ControlValueNone,
	ControlValueBool,
	ControlValueInteger,
	ControlValueInteger64,
};

class ControlValue
{
public:
	ControlValue();
	ControlValue(bool value);
	ControlValue(int value);
	ControlValue(int64_t value);

	ControlValueType type() const { return type_; };
	bool isNone() const { return type_ == ControlValueNone; };

	void set(bool value);
	void set(int value);
	void set(int64_t value);

	bool getBool() const;
	int getInt() const;
	int64_t getInt64() const;

	std::string toString() const;

private:
	ControlValueType type_;

	union {
		bool bool_;
		int integer_;
		int64_t integer64_;
	};
};

struct ControlIdentifier {
	ControlId id;
	const char *name;
	ControlValueType type;
};

class ControlInfo
{
public:
	explicit ControlInfo(ControlId id, const ControlValue &min = 0,
			     const ControlValue &max = 0);

	ControlId id() const { return ident_->id; }
	const char *name() const { return ident_->name; }
	ControlValueType type() const { return ident_->type; }

	const ControlValue &min() const { return min_; }
	const ControlValue &max() const { return max_; }

	std::string toString() const;

private:
	const struct ControlIdentifier *ident_;
	ControlValue min_;
	ControlValue max_;
};

bool operator==(const ControlInfo &lhs, const ControlInfo &rhs);
bool operator==(const ControlId &lhs, const ControlInfo &rhs);
bool operator==(const ControlInfo &lhs, const ControlId &rhs);
static inline bool operator!=(const ControlInfo &lhs, const ControlInfo &rhs)
{
	return !(lhs == rhs);
}
static inline bool operator!=(const ControlId &lhs, const ControlInfo &rhs)
{
	return !(lhs == rhs);
}
static inline bool operator!=(const ControlInfo &lhs, const ControlId &rhs)
{
	return !(lhs == rhs);
}

class ControlList
{
private:
	using ControlListMap = std::unordered_map<const ControlInfo *, ControlValue>;

public:
	ControlList(Camera *camera);

	using iterator = ControlListMap::iterator;
	using const_iterator = ControlListMap::const_iterator;

	iterator begin() { return controls_.begin(); }
	iterator end() { return controls_.end(); }
	const_iterator begin() const { return controls_.begin(); }
	const_iterator end() const { return controls_.end(); }

	bool contains(const ControlInfo *info) const;
	bool empty() const { return controls_.empty(); }
	std::size_t size() const { return controls_.size(); }
	void clear() { controls_.clear(); }

	ControlValue &operator[](const ControlInfo *info) { return controls_[info]; }

	void update(const ControlList &list);

private:
	Camera *camera_;
	ControlListMap controls_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CONTROLS_H__ */
