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

class ControlValidator;

enum ControlType {
	ControlTypeNone,
	ControlTypeBool,
	ControlTypeInteger32,
	ControlTypeInteger64,
};

namespace details {

template<typename T>
struct control_type {
};

template<>
struct control_type<void> {
	static constexpr ControlType value = ControlTypeNone;
};

template<>
struct control_type<bool> {
	static constexpr ControlType value = ControlTypeBool;
};

template<>
struct control_type<int32_t> {
	static constexpr ControlType value = ControlTypeInteger32;
};

template<>
struct control_type<int64_t> {
	static constexpr ControlType value = ControlTypeInteger64;
};

} /* namespace details */

class ControlValue
{
public:
	ControlValue();
	ControlValue(bool value);
	ControlValue(int32_t value);
	ControlValue(int64_t value);

	ControlType type() const { return type_; }
	bool isNone() const { return type_ == ControlTypeNone; }

	std::string toString() const;

	bool operator==(const ControlValue &other) const;
	bool operator!=(const ControlValue &other) const
	{
		return !(*this == other);
	}

	template<typename T>
	T get() const;
	template<typename T>
	void set(const T &value);

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
	ControlId(unsigned int id, const std::string &name, ControlType type)
		: id_(id), name_(name), type_(type)
	{
	}

	unsigned int id() const { return id_; }
	const std::string &name() const { return name_; }
	ControlType type() const { return type_; }

private:
	ControlId &operator=(const ControlId &) = delete;
	ControlId(const ControlId &) = delete;

	unsigned int id_;
	std::string name_;
	ControlType type_;
};

static inline bool operator==(unsigned int lhs, const ControlId &rhs)
{
	return lhs == rhs.id();
}

static inline bool operator!=(unsigned int lhs, const ControlId &rhs)
{
	return !(lhs == rhs);
}

static inline bool operator==(const ControlId &lhs, unsigned int rhs)
{
	return lhs.id() == rhs;
}

static inline bool operator!=(const ControlId &lhs, unsigned int rhs)
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

class ControlRange
{
public:
	explicit ControlRange(const ControlValue &min = 0,
			      const ControlValue &max = 0,
			      const ControlValue &def = 0);

	const ControlValue &min() const { return min_; }
	const ControlValue &max() const { return max_; }
	const ControlValue &def() const { return def_; }

	std::string toString() const;

	bool operator==(const ControlRange &other) const
	{
		return min_ == other.min_ && max_ == other.max_;
	}

	bool operator!=(const ControlRange &other) const
	{
		return !(*this == other);
	}

private:
	ControlValue min_;
	ControlValue max_;
	ControlValue def_;
};

using ControlIdMap = std::unordered_map<unsigned int, const ControlId *>;

class ControlInfoMap : private std::unordered_map<const ControlId *, ControlRange>
{
public:
	using Map = std::unordered_map<const ControlId *, ControlRange>;

	ControlInfoMap() = default;
	ControlInfoMap(const ControlInfoMap &other) = default;
	ControlInfoMap(std::initializer_list<Map::value_type> init);
	ControlInfoMap(Map &&info);

	ControlInfoMap &operator=(const ControlInfoMap &other) = default;
	ControlInfoMap &operator=(std::initializer_list<Map::value_type> init);
	ControlInfoMap &operator=(Map &&info);

	using Map::key_type;
	using Map::mapped_type;
	using Map::value_type;
	using Map::size_type;
	using Map::iterator;
	using Map::const_iterator;

	using Map::begin;
	using Map::cbegin;
	using Map::end;
	using Map::cend;
	using Map::at;
	using Map::empty;
	using Map::size;
	using Map::count;
	using Map::find;

	mapped_type &at(unsigned int key);
	const mapped_type &at(unsigned int key) const;
	size_type count(unsigned int key) const;
	iterator find(unsigned int key);
	const_iterator find(unsigned int key) const;

	const ControlIdMap &idmap() const { return idmap_; }

private:
	void generateIdmap();

	ControlIdMap idmap_;
};

class ControlList
{
private:
	using ControlListMap = std::unordered_map<unsigned int, ControlValue>;

public:
	ControlList();
	ControlList(const ControlIdMap &idmap, ControlValidator *validator = nullptr);
	ControlList(const ControlInfoMap &info, ControlValidator *validator = nullptr);

	using iterator = ControlListMap::iterator;
	using const_iterator = ControlListMap::const_iterator;

	iterator begin() { return controls_.begin(); }
	iterator end() { return controls_.end(); }
	const_iterator begin() const { return controls_.begin(); }
	const_iterator end() const { return controls_.end(); }

	bool empty() const { return controls_.empty(); }
	std::size_t size() const { return controls_.size(); }
	void clear() { controls_.clear(); }

	bool contains(const ControlId &id) const;
	bool contains(unsigned int id) const;

	template<typename T>
	T get(const Control<T> &ctrl) const
	{
		const ControlValue *val = find(ctrl.id());
		if (!val)
			return T{};

		return val->get<T>();
	}

	template<typename T, typename V>
	void set(const Control<T> &ctrl, const V &value)
	{
		ControlValue *val = find(ctrl.id());
		if (!val)
			return;

		val->set<T>(value);
	}

	const ControlValue &get(unsigned int id) const;
	void set(unsigned int id, const ControlValue &value);

	const ControlInfoMap *infoMap() const { return infoMap_; }

private:
	const ControlValue *find(unsigned int id) const;
	ControlValue *find(unsigned int id);

	ControlValidator *validator_;
	const ControlIdMap *idmap_;
	const ControlInfoMap *infoMap_;

	ControlListMap controls_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_CONTROLS_H__ */
