/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 * Copyright (C) 2026, Ideas on Board
 *
 * Data structure to manage tree of values
 */

#pragma once

#include <initializer_list>
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

class ValueNode
{
private:
	struct Value {
		Value(std::string k, std::unique_ptr<ValueNode> &&v)
			: key(std::move(k)), value(std::move(v))
		{
		}
		std::string key;
		std::unique_ptr<ValueNode> value;
	};

	using ValueContainer = std::vector<Value>;

public:
#ifndef __DOXYGEN__
	template<typename Derived, typename ContainerIterator>
	class Iterator
	{
	public:
		using difference_type = std::ptrdiff_t;
		using iterator_category = std::forward_iterator_tag;

		Iterator(ContainerIterator it)
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
		ContainerIterator it_;
	};

	template<typename Iterator, typename Container>
	class Adapter
	{
	public:
		Adapter(Container &container)
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
		Container &container_;
	};

	template<typename Value, typename ContainerIterator>
	class ListIterator : public Iterator<ListIterator<Value, ContainerIterator>,
					     ContainerIterator>
	{
	private:
		using Base = Iterator<ListIterator<Value, ContainerIterator>,
				      ContainerIterator>;

	public:
		using value_type = Value;
		using pointer = value_type *;
		using reference = value_type &;

		reference operator*() const
		{
			return *Base::it_->value.get();
		}

		pointer operator->() const
		{
			return Base::it_->value.get();
		}
	};

	template<typename Value, typename ContainerIterator>
	class DictIterator : public Iterator<DictIterator<Value, ContainerIterator>,
					     ContainerIterator>
	{
	private:
		using Base = Iterator<DictIterator<Value, ContainerIterator>,
				      ContainerIterator>;

	public:
		using value_type = std::pair<const std::string &, Value &>;
		using pointer = value_type *;
		using reference = value_type &;

		value_type operator*() const
		{
			return { Base::it_->key, *Base::it_->value.get() };
		}
	};

	class DictAdapter : public Adapter<DictIterator<ValueNode,
							ValueContainer::iterator>,
					   ValueContainer>
	{
	public:
		using key_type = std::string;
	};

	class ListAdapter : public Adapter<ListIterator<ValueNode,
							ValueContainer::iterator>,
					   ValueContainer>
	{
	};

	class ConstDictAdapter : public Adapter<DictIterator<const ValueNode,
							     ValueContainer::const_iterator>,
						const ValueContainer>
	{
	public:
		using key_type = std::string;
	};

	class ConstListAdapter : public Adapter<ListIterator<const ValueNode,
							     ValueContainer::const_iterator>,
						const ValueContainer>
	{
	};
#endif /* __DOXYGEN__ */

	ValueNode();

	template<typename T>
	ValueNode(T &&value)
		: type_(Type::Empty)
	{
		set(std::forward<T>(value));
	}

	~ValueNode();

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

	DictAdapter asDict() { return DictAdapter{ list_ }; }
	ListAdapter asList() { return ListAdapter{ list_ }; }
	ConstDictAdapter asDict() const { return ConstDictAdapter{ list_ }; }
	ConstListAdapter asList() const { return ConstListAdapter{ list_ }; }

	ValueNode *at(std::size_t index);
	const ValueNode &operator[](std::size_t index) const;

	bool contains(std::string_view key) const;
	ValueNode *at(std::string_view key);
	const ValueNode &operator[](std::string_view key) const;
	const ValueNode &operator[](std::initializer_list<std::string_view> path) const;

	ValueNode *add(std::unique_ptr<ValueNode> &&child);
	ValueNode *add(std::string key, std::unique_ptr<ValueNode> &&child);
	ValueNode *add(std::initializer_list<std::string_view> path,
		       std::unique_ptr<ValueNode> &&child);

	void erase(std::string_view key);
	void erase(std::initializer_list<std::string_view> path);

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(ValueNode)

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
		std::optional<T> get(const ValueNode &obj) const;
		void set(ValueNode &obj, T value);
	};

	Type type_;

	std::string value_;
	ValueContainer list_;
	std::map<std::string, ValueNode *, std::less<>> dictionary_;
};

} /* namespace libcamera */
