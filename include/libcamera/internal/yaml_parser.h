/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * yaml_parser.h - libcamera YAML parsing helper
 */

#pragma once

#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <libcamera/base/class.h>

#include <libcamera/geometry.h>

namespace libcamera {

class File;
class YamlParserContext;

class YamlObject
{
private:
	struct Value {
		Value(std::string &&k, std::unique_ptr<YamlObject> &&v)
			: key(std::move(k)), value(std::move(v))
		{
		}
		std::string key;
		std::unique_ptr<YamlObject> value;
	};

	using Container = std::vector<Value>;
	using ListContainer = std::vector<std::unique_ptr<YamlObject>>;

public:
#ifndef __DOXYGEN__
	template<typename Derived>
	class Iterator
	{
	public:
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		Iterator(typename Container::const_iterator it)
			: it_(it)
		{
		}

		Derived &operator++()
		{
			++it_;
			return *static_cast<Derived *>(this);
		}

		Derived operator++(int)
		{
			Derived it = *static_cast<Derived *>(this);
			it_++;
			return it;
		}

		friend bool operator==(const Iterator &a, const Iterator &b)
		{
			return a.it_ == b.it_;
		}

		friend bool operator!=(const Iterator &a, const Iterator &b)
		{
			return a.it_ != b.it_;
		}

	protected:
		Container::const_iterator it_;
	};

	template<typename Iterator>
	class Adapter
	{
	public:
		Adapter(const Container &container)
			: container_(container)
		{
		}

		Iterator begin() const
		{
			return Iterator{ container_.begin() };
		}

		Iterator end() const
		{
			return Iterator{ container_.end() };
		}

	protected:
		const Container &container_;
	};

	class ListIterator : public Iterator<ListIterator>
	{
	public:
		using value_type = const YamlObject &;
		using pointer = const YamlObject *;
		using reference = value_type;

		value_type operator*() const
		{
			return *it_->value.get();
		}

		pointer operator->() const
		{
			return it_->value.get();
		}
	};

	class DictIterator : public Iterator<DictIterator>
	{
	public:
		using value_type = std::pair<const std::string &, const YamlObject &>;
		using pointer = value_type *;
		using reference = value_type &;

		value_type operator*() const
		{
			return { it_->key, *it_->value.get() };
		}
	};

	class DictAdapter : public Adapter<DictIterator>
	{
	public:
		using key_type = std::string;
	};

	class ListAdapter : public Adapter<ListIterator>
	{
	};
#endif /* __DOXYGEN__ */

	YamlObject();
	~YamlObject();

	bool isValue() const
	{
		return type_ == Type::Value;
	}
	bool isList() const
	{
		return type_ == Type::List;
	}
	bool isDictionary() const
	{
		return type_ == Type::Dictionary;
	}

	std::size_t size() const;

#ifndef __DOXYGEN__
	template<typename T,
		 std::enable_if_t<
			 std::is_same_v<bool, T> ||
			 std::is_same_v<double, T> ||
			 std::is_same_v<int8_t, T> ||
			 std::is_same_v<uint8_t, T> ||
			 std::is_same_v<int16_t, T> ||
			 std::is_same_v<uint16_t, T> ||
			 std::is_same_v<int32_t, T> ||
			 std::is_same_v<uint32_t, T> ||
			 std::is_same_v<std::string, T> ||
			 std::is_same_v<Size, T>> * = nullptr>
#else
	template<typename T>
#endif
	std::optional<T> get() const;

	template<typename T>
	T get(const T &defaultValue) const
	{
		return get<T>().value_or(defaultValue);
	}

#ifndef __DOXYGEN__
	template<typename T,
		 std::enable_if_t<
			 std::is_same_v<bool, T> ||
			 std::is_same_v<double, T> ||
			 std::is_same_v<int8_t, T> ||
			 std::is_same_v<uint8_t, T> ||
			 std::is_same_v<int16_t, T> ||
			 std::is_same_v<uint16_t, T> ||
			 std::is_same_v<int32_t, T> ||
			 std::is_same_v<uint32_t, T> ||
			 std::is_same_v<std::string, T> ||
			 std::is_same_v<Size, T>> * = nullptr>
#else
	template<typename T>
#endif
	std::optional<std::vector<T>> getList() const;

	DictAdapter asDict() const { return DictAdapter{ list_ }; }
	ListAdapter asList() const { return ListAdapter{ list_ }; }

	const YamlObject &operator[](std::size_t index) const;

	bool contains(const std::string &key) const;
	const YamlObject &operator[](const std::string &key) const;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(YamlObject)

	friend class YamlParserContext;

	enum class Type {
		Dictionary,
		List,
		Value,
	};

	Type type_;

	std::string value_;
	Container list_;
	std::map<std::string, YamlObject *> dictionary_;
};

class YamlParser final
{
public:
	static std::unique_ptr<YamlObject> parse(File &file);
};

} /* namespace libcamera */
