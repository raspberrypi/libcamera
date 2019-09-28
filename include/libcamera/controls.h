/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.h - Control handling
 */

#ifndef __LIBCAMERA_CONTROLS_H__
#define __LIBCAMERA_CONTROLS_H__

#include <string>
#include <unordered_map>

namespace libcamera {

class Camera;

enum ControlType {
	ControlTypeNone,
	ControlTypeBool,
	ControlTypeInteger32,
	ControlTypeInteger64,
};

class ControlValue
{
public:
	ControlValue();
	ControlValue(bool value);
	ControlValue(int32_t value);
	ControlValue(int64_t value);

	ControlType type() const { return type_; };
	bool isNone() const { return type_ == ControlTypeNone; };

	template<typename T>
	const T &get() const;
	template<typename T>
	void set(const T &value);

	std::string toString() const;

private:
	ControlType type_;

	union {
		bool bool_;
		int32_t integer32_;
		int64_t integer64_;
	};
};

class ControlId
{
public:
	unsigned int id() const { return id_; }
	const char *name() const { return name_; }
	ControlType type() const { return type_; }

protected:
	ControlId(unsigned int id, const char *name, ControlType type)
		: id_(id), name_(name), type_(type)
	{
	}

private:
	ControlId(const ControlId &) = delete;
	ControlId &operator=(const ControlId &) = delete;

	unsigned int id_;
	const char *name_;
	ControlType type_;
};

static inline bool operator==(const ControlId &lhs, const ControlId &rhs)
{
	return lhs.id() == rhs.id();
}

static inline bool operator!=(const ControlId &lhs, const ControlId &rhs)
{
	return !(lhs == rhs);
}

template<typename T>
class Control : public ControlId
{
public:
	using type = T;

	Control(unsigned int id, const char *name);

private:
	Control(const Control &) = delete;
	Control &operator=(const Control &) = delete;
};

class ControlInfo
{
public:
	explicit ControlInfo(const ControlValue &min = 0,
			     const ControlValue &max = 0);

	const ControlValue &min() const { return min_; }
	const ControlValue &max() const { return max_; }

	std::string toString() const;

private:
	ControlValue min_;
	ControlValue max_;
};

using ControlInfoMap = std::unordered_map<const ControlId *, ControlInfo>;

class ControlList
{
private:
	using ControlListMap = std::unordered_map<const ControlId *, ControlValue>;

public:
	ControlList(Camera *camera);

	using iterator = ControlListMap::iterator;
	using const_iterator = ControlListMap::const_iterator;

	iterator begin() { return controls_.begin(); }
	iterator end() { return controls_.end(); }
	const_iterator begin() const { return controls_.begin(); }
	const_iterator end() const { return controls_.end(); }

	bool contains(const ControlId &id) const;
	bool empty() const { return controls_.empty(); }
	std::size_t size() const { return controls_.size(); }
	void clear() { controls_.clear(); }

	template<typename T>
	const T &get(const Control<T> &ctrl) const
	{
		const ControlValue *val = find(ctrl);
		if (!val) {
			static T t(0);
			return t;
		}

		return val->get<T>();
	}

	template<typename T>
	void set(const Control<T> &ctrl, const T &value)
	{
		ControlValue *val = find(ctrl);
		if (!val)
			return;

		val->set<T>(value);
	}

private:
	const ControlValue *find(const ControlId &id) const;
	ControlValue *find(const ControlId &id);

	Camera *camera_;
	ControlListMap controls_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CONTROLS_H__ */
