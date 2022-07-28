/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * bound_method.h - Method bind and invocation
 */

#pragma once

#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>

namespace libcamera {

class Object;

enum ConnectionType {
	ConnectionTypeAuto,
	ConnectionTypeDirect,
	ConnectionTypeQueued,
	ConnectionTypeBlocking,
};

class BoundMethodPackBase
{
public:
	virtual ~BoundMethodPackBase() = default;
};

template<typename R, typename... Args>
class BoundMethodPack : public BoundMethodPackBase
{
public:
	BoundMethodPack(const Args &... args)
		: args_(args...)
	{
	}

	R returnValue()
	{
		return ret_;
	}

	std::tuple<typename std::remove_reference_t<Args>...> args_;
	R ret_;
};

template<typename... Args>
class BoundMethodPack<void, Args...> : public BoundMethodPackBase
{
public:
	BoundMethodPack(const Args &... args)
		: args_(args...)
	{
	}

	void returnValue()
	{
	}

	std::tuple<typename std::remove_reference_t<Args>...> args_;
};

class BoundMethodBase
{
public:
	BoundMethodBase(void *obj, Object *object, ConnectionType type)
		: obj_(obj), object_(object), connectionType_(type)
	{
	}
	virtual ~BoundMethodBase() = default;

	template<typename T, std::enable_if_t<!std::is_same<Object, T>::value> * = nullptr>
	bool match(T *obj) { return obj == obj_; }
	bool match(Object *object) { return object == object_; }

	Object *object() const { return object_; }

	virtual void invokePack(BoundMethodPackBase *pack) = 0;

protected:
	bool activatePack(std::shared_ptr<BoundMethodPackBase> pack,
			  bool deleteMethod);

	void *obj_;
	Object *object_;

private:
	ConnectionType connectionType_;
};

template<typename R, typename... Args>
class BoundMethodArgs : public BoundMethodBase
{
public:
	using PackType = BoundMethodPack<R, Args...>;

private:
	template<std::size_t... I, typename T = R>
	std::enable_if_t<!std::is_void<T>::value, void>
	invokePack(BoundMethodPackBase *pack, std::index_sequence<I...>)
	{
		PackType *args = static_cast<PackType *>(pack);
		args->ret_ = invoke(std::get<I>(args->args_)...);
	}

	template<std::size_t... I, typename T = R>
	std::enable_if_t<std::is_void<T>::value, void>
	invokePack(BoundMethodPackBase *pack, std::index_sequence<I...>)
	{
		/* args is effectively unused when the sequence I is empty. */
		PackType *args [[gnu::unused]] = static_cast<PackType *>(pack);
		invoke(std::get<I>(args->args_)...);
	}

public:
	BoundMethodArgs(void *obj, Object *object, ConnectionType type)
		: BoundMethodBase(obj, object, type) {}

	void invokePack(BoundMethodPackBase *pack) override
	{
		invokePack(pack, std::make_index_sequence<sizeof...(Args)>{});
	}

	virtual R activate(Args... args, bool deleteMethod = false) = 0;
	virtual R invoke(Args... args) = 0;
};

template<typename T, typename R, typename Func, typename... Args>
class BoundMethodFunctor : public BoundMethodArgs<R, Args...>
{
public:
	using PackType = typename BoundMethodArgs<R, Args...>::PackType;

	BoundMethodFunctor(T *obj, Object *object, Func func,
			   ConnectionType type = ConnectionTypeAuto)
		: BoundMethodArgs<R, Args...>(obj, object, type), func_(func)
	{
	}

	R activate(Args... args, bool deleteMethod = false) override
	{
		if (!this->object_)
			return func_(args...);

		auto pack = std::make_shared<PackType>(args...);
		bool sync = BoundMethodBase::activatePack(pack, deleteMethod);
		return sync ? pack->returnValue() : R();
	}

	R invoke(Args... args) override
	{
		return func_(args...);
	}

private:
	Func func_;
};

template<typename T, typename R, typename... Args>
class BoundMethodMember : public BoundMethodArgs<R, Args...>
{
public:
	using PackType = typename BoundMethodArgs<R, Args...>::PackType;

	BoundMethodMember(T *obj, Object *object, R (T::*func)(Args...),
			  ConnectionType type = ConnectionTypeAuto)
		: BoundMethodArgs<R, Args...>(obj, object, type), func_(func)
	{
	}

	bool match(R (T::*func)(Args...)) const { return func == func_; }

	R activate(Args... args, bool deleteMethod = false) override
	{
		if (!this->object_) {
			T *obj = static_cast<T *>(this->obj_);
			return (obj->*func_)(args...);
		}

		auto pack = std::make_shared<PackType>(args...);
		bool sync = BoundMethodBase::activatePack(pack, deleteMethod);
		return sync ? pack->returnValue() : R();
	}

	R invoke(Args... args) override
	{
		T *obj = static_cast<T *>(this->obj_);
		return (obj->*func_)(args...);
	}

private:
	R (T::*func_)(Args...);
};

template<typename R, typename... Args>
class BoundMethodStatic : public BoundMethodArgs<R, Args...>
{
public:
	BoundMethodStatic(R (*func)(Args...))
		: BoundMethodArgs<R, Args...>(nullptr, nullptr, ConnectionTypeAuto),
		  func_(func)
	{
	}

	bool match(R (*func)(Args...)) const { return func == func_; }

	R activate(Args... args, [[maybe_unused]] bool deleteMethod = false) override
	{
		return (*func_)(args...);
	}

	R invoke(Args...) override
	{
		return R();
	}

private:
	R (*func_)(Args...);
};

} /* namespace libcamera */
