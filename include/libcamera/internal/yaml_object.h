/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 * Copyright (C) 2026, Ideas on Board
 *
 * libcamera YAML object
 */

#pragma once

#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <libcamera/base/class.h>

namespace libcamera {

class YamlObject
{
private:
	struct Value {
		Value(std::string k, std::unique_ptr<YamlObject> &&v)
			: key(std::move(k)), value(std::move(v))
		{
		}
		std::string key;
		std::unique_ptr<YamlObject> value;
	};

	using ValueContainer = std::vector<Value>;

public:
#ifndef __DOXYGEN__
	template<typename Derived>
	class Iterator
	{
	public:
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		Iterator(typename ValueContainer::const_iterator it)
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
		ValueContainer::const_iterator it_;
	};

	template<typename Iterator>
	class Adapter
	{
	public:
		Adapter(const ValueContainer &container)
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
		const ValueContainer &container_;
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
	bool isEmpty() const
	{
		return type_ == Type::Empty;
	}
	explicit operator bool() const
	{
		return type_ != Type::Empty;
	}

	std::size_t size() const;

	template<typename T>
	std::optional<T> get() const
	{
		return Accessor<T>{}.get(*this);
	}

	template<typename T, typename U>
	T get(U &&defaultValue) const
	{
		return get<T>().value_or(std::forward<U>(defaultValue));
	}

	template<typename T>
	void set(T &&value)
	{
		return Accessor<std::remove_cv_t<std::remove_reference_t<T>>>{}
			.set(*this, std::forward<T>(value));
	}

	DictAdapter asDict() const { return DictAdapter{ list_ }; }
	ListAdapter asList() const { return ListAdapter{ list_ }; }

	const YamlObject &operator[](std::size_t index) const;

	bool contains(std::string_view key) const;
	const YamlObject &operator[](std::string_view key) const;

	YamlObject *add(std::unique_ptr<YamlObject> &&child);
	YamlObject *add(std::string key, std::unique_ptr<YamlObject> &&child);

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(YamlObject)

	template<typename T>
	friend struct Accessor;

	enum class Type {
		Dictionary,
		List,
		Value,
		Empty,
	};

	template<typename T, typename Enable = void>
	struct Accessor {
		std::optional<T> get(const YamlObject &obj) const;
		void set(YamlObject &obj, T value);
	};

	Type type_;

	std::string value_;
	ValueContainer list_;
	std::map<std::string, YamlObject *, std::less<>> dictionary_;
};

} /* namespace libcamera */
