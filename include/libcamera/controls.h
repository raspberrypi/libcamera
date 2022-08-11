/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * controls.h - Control handling
 */

#pragma once

#include <assert.h>
#include <optional>
#include <set>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/span.h>

#include <libcamera/geometry.h>

namespace libcamera {

class ControlValidator;

enum ControlType {
	ControlTypeNone,
	ControlTypeBool,
	ControlTypeByte,
	ControlTypeInteger32,
	ControlTypeInteger64,
	ControlTypeFloat,
	ControlTypeString,
	ControlTypeRectangle,
	ControlTypeSize,
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
struct control_type<uint8_t> {
	static constexpr ControlType value = ControlTypeByte;
};

template<>
struct control_type<int32_t> {
	static constexpr ControlType value = ControlTypeInteger32;
};

template<>
struct control_type<int64_t> {
	static constexpr ControlType value = ControlTypeInteger64;
};

template<>
struct control_type<float> {
	static constexpr ControlType value = ControlTypeFloat;
};

template<>
struct control_type<std::string> {
	static constexpr ControlType value = ControlTypeString;
};

template<>
struct control_type<Rectangle> {
	static constexpr ControlType value = ControlTypeRectangle;
};

template<>
struct control_type<Size> {
	static constexpr ControlType value = ControlTypeSize;
};

template<typename T, std::size_t N>
struct control_type<Span<T, N>> : public control_type<std::remove_cv_t<T>> {
};

} /* namespace details */

class ControlValue
{
public:
	ControlValue();

#ifndef __DOXYGEN__
	template<typename T, std::enable_if_t<!details::is_span<T>::value &&
					      details::control_type<T>::value &&
					      !std::is_same<std::string, std::remove_cv_t<T>>::value,
					      std::nullptr_t> = nullptr>
	ControlValue(const T &value)
		: type_(ControlTypeNone), numElements_(0)
	{
		set(details::control_type<std::remove_cv_t<T>>::value, false,
		    &value, 1, sizeof(T));
	}

	template<typename T, std::enable_if_t<details::is_span<T>::value ||
					      std::is_same<std::string, std::remove_cv_t<T>>::value,
					      std::nullptr_t> = nullptr>
#else
	template<typename T>
#endif
	ControlValue(const T &value)
		: type_(ControlTypeNone), numElements_(0)
	{
		set(details::control_type<std::remove_cv_t<T>>::value, true,
		    value.data(), value.size(), sizeof(typename T::value_type));
	}

	~ControlValue();

	ControlValue(const ControlValue &other);
	ControlValue &operator=(const ControlValue &other);

	ControlType type() const { return type_; }
	bool isNone() const { return type_ == ControlTypeNone; }
	bool isArray() const { return isArray_; }
	std::size_t numElements() const { return numElements_; }
	Span<const uint8_t> data() const;
	Span<uint8_t> data();

	std::string toString() const;

	bool operator==(const ControlValue &other) const;
	bool operator!=(const ControlValue &other) const
	{
		return !(*this == other);
	}

#ifndef __DOXYGEN__
	template<typename T, std::enable_if_t<!details::is_span<T>::value &&
					      !std::is_same<std::string, std::remove_cv_t<T>>::value,
					      std::nullptr_t> = nullptr>
	T get() const
	{
		assert(type_ == details::control_type<std::remove_cv_t<T>>::value);
		assert(!isArray_);

		return *reinterpret_cast<const T *>(data().data());
	}

	template<typename T, std::enable_if_t<details::is_span<T>::value ||
					      std::is_same<std::string, std::remove_cv_t<T>>::value,
					      std::nullptr_t> = nullptr>
#else
	template<typename T>
#endif
	T get() const
	{
		assert(type_ == details::control_type<std::remove_cv_t<T>>::value);
		assert(isArray_);

		using V = typename T::value_type;
		const V *value = reinterpret_cast<const V *>(data().data());
		return T{ value, numElements_ };
	}

#ifndef __DOXYGEN__
	template<typename T, std::enable_if_t<!details::is_span<T>::value &&
					      !std::is_same<std::string, std::remove_cv_t<T>>::value,
					      std::nullptr_t> = nullptr>
	void set(const T &value)
	{
		set(details::control_type<std::remove_cv_t<T>>::value, false,
		    reinterpret_cast<const void *>(&value), 1, sizeof(T));
	}

	template<typename T, std::enable_if_t<details::is_span<T>::value ||
					      std::is_same<std::string, std::remove_cv_t<T>>::value,
					      std::nullptr_t> = nullptr>
#else
	template<typename T>
#endif
	void set(const T &value)
	{
		set(details::control_type<std::remove_cv_t<T>>::value, true,
		    value.data(), value.size(), sizeof(typename T::value_type));
	}

	void reserve(ControlType type, bool isArray = false,
		     std::size_t numElements = 1);

private:
	ControlType type_ : 8;
	bool isArray_;
	std::size_t numElements_ : 32;
	union {
		uint64_t value_;
		void *storage_;
	};

	void release();
	void set(ControlType type, bool isArray, const void *data,
		 std::size_t numElements, std::size_t elementSize);
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
	LIBCAMERA_DISABLE_COPY_AND_MOVE(ControlId)

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

	Control(unsigned int id, const char *name)
		: ControlId(id, name, details::control_type<std::remove_cv_t<T>>::value)
	{
	}

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(Control)
};

class ControlInfo
{
public:
	explicit ControlInfo(const ControlValue &min = {},
			     const ControlValue &max = {},
			     const ControlValue &def = {});
	explicit ControlInfo(Span<const ControlValue> values,
			     const ControlValue &def = {});
	explicit ControlInfo(std::set<bool> values, bool def);
	explicit ControlInfo(bool value);

	const ControlValue &min() const { return min_; }
	const ControlValue &max() const { return max_; }
	const ControlValue &def() const { return def_; }
	const std::vector<ControlValue> &values() const { return values_; }

	std::string toString() const;

	bool operator==(const ControlInfo &other) const
	{
		return min_ == other.min_ && max_ == other.max_;
	}

	bool operator!=(const ControlInfo &other) const
	{
		return !(*this == other);
	}

private:
	ControlValue min_;
	ControlValue max_;
	ControlValue def_;
	std::vector<ControlValue> values_;
};

using ControlIdMap = std::unordered_map<unsigned int, const ControlId *>;

class ControlInfoMap : private std::unordered_map<const ControlId *, ControlInfo>
{
public:
	using Map = std::unordered_map<const ControlId *, ControlInfo>;

	ControlInfoMap() = default;
	ControlInfoMap(const ControlInfoMap &other) = default;
	ControlInfoMap(std::initializer_list<Map::value_type> init,
		       const ControlIdMap &idmap);
	ControlInfoMap(Map &&info, const ControlIdMap &idmap);

	ControlInfoMap &operator=(const ControlInfoMap &other) = default;

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

	const ControlIdMap &idmap() const { return *idmap_; }

private:
	bool validate();

	const ControlIdMap *idmap_ = nullptr;
};

class ControlList
{
private:
	using ControlListMap = std::unordered_map<unsigned int, ControlValue>;

public:
	ControlList();
	ControlList(const ControlIdMap &idmap, const ControlValidator *validator = nullptr);
	ControlList(const ControlInfoMap &infoMap, const ControlValidator *validator = nullptr);

	using iterator = ControlListMap::iterator;
	using const_iterator = ControlListMap::const_iterator;

	iterator begin() { return controls_.begin(); }
	iterator end() { return controls_.end(); }
	const_iterator begin() const { return controls_.begin(); }
	const_iterator end() const { return controls_.end(); }

	bool empty() const { return controls_.empty(); }
	std::size_t size() const { return controls_.size(); }

	void clear() { controls_.clear(); }
	void merge(const ControlList &source);

	bool contains(unsigned int id) const;

	template<typename T>
	std::optional<T> get(const Control<T> &ctrl) const
	{
		const auto entry = controls_.find(ctrl.id());
		if (entry == controls_.end())
			return std::nullopt;

		const ControlValue &val = entry->second;
		return val.get<T>();
	}

	template<typename T, typename V>
	void set(const Control<T> &ctrl, const V &value)
	{
		ControlValue *val = find(ctrl.id());
		if (!val)
			return;

		val->set<T>(value);
	}

	template<typename T, typename V, size_t Size>
	void set(const Control<Span<T, Size>> &ctrl, const std::initializer_list<V> &value)
	{
		ControlValue *val = find(ctrl.id());
		if (!val)
			return;

		val->set(Span<const typename std::remove_cv_t<V>, Size>{ value.begin(), value.size() });
	}

	const ControlValue &get(unsigned int id) const;
	void set(unsigned int id, const ControlValue &value);

	const ControlInfoMap *infoMap() const { return infoMap_; }
	const ControlIdMap *idMap() const { return idmap_; }

private:
	const ControlValue *find(unsigned int id) const;
	ControlValue *find(unsigned int id);

	const ControlValidator *validator_;
	const ControlIdMap *idmap_;
	const ControlInfoMap *infoMap_;

	ControlListMap controls_;
};

} /* namespace libcamera */
