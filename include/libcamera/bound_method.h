/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * bound_method.h - Method bind and invocation
 */
#ifndef __LIBCAMERA_BOUND_METHOD_H__
#define __LIBCAMERA_BOUND_METHOD_H__

#include <tuple>
#include <type_traits>

namespace libcamera {

class Object;

enum ConnectionType {
	ConnectionTypeAuto,
	ConnectionTypeDirect,
	ConnectionTypeQueued,
	ConnectionTypeBlocking,
};

class BoundMethodBase
{
public:
	BoundMethodBase(void *obj, Object *object, ConnectionType type)
		: obj_(obj), object_(object), connectionType_(type)
	{
	}
	virtual ~BoundMethodBase() {}

	template<typename T, typename std::enable_if<!std::is_same<Object, T>::value>::type * = nullptr>
	bool match(T *obj) { return obj == obj_; }
	bool match(Object *object) { return object == object_; }

	Object *object() const { return object_; }

	virtual void invokePack(void *pack) = 0;

protected:
#ifndef __DOXYGEN__
	/*
	 * This is a cheap partial implementation of std::integer_sequence<>
	 * from C++14.
	 */
	template<int...>
	struct sequence {
	};

	template<int N, int... S>
	struct generator : generator<N-1, N-1, S...> {
	};

	template<int... S>
	struct generator<0, S...> {
		typedef sequence<S...> type;
	};
#endif

	void activatePack(void *pack, bool deleteMethod);

	void *obj_;
	Object *object_;

private:
	ConnectionType connectionType_;
};

template<typename... Args>
class BoundMethodArgs : public BoundMethodBase
{
private:
	using PackType = std::tuple<typename std::remove_reference<Args>::type...>;

	template<int... S>
	void invokePack(void *pack, BoundMethodBase::sequence<S...>)
	{
		PackType *args = static_cast<PackType *>(pack);
		invoke(std::get<S>(*args)...);
		delete args;
	}

public:
	BoundMethodArgs(void *obj, Object *object, ConnectionType type)
		: BoundMethodBase(obj, object, type) {}

	void invokePack(void *pack) override
	{
		invokePack(pack, typename BoundMethodBase::generator<sizeof...(Args)>::type());
	}

	virtual void activate(Args... args, bool deleteMethod = false) = 0;
	virtual void invoke(Args... args) = 0;
};

template<typename T, typename... Args>
class BoundMemberMethod : public BoundMethodArgs<Args...>
{
public:
	using PackType = std::tuple<typename std::remove_reference<Args>::type...>;

	BoundMemberMethod(T *obj, Object *object, void (T::*func)(Args...),
			  ConnectionType type = ConnectionTypeAuto)
		: BoundMethodArgs<Args...>(obj, object, type), func_(func)
	{
	}

	bool match(void (T::*func)(Args...)) const { return func == func_; }

	void activate(Args... args, bool deleteMethod = false) override
	{
		if (this->object_)
			BoundMethodBase::activatePack(new PackType{ args... }, deleteMethod);
		else
			(static_cast<T *>(this->obj_)->*func_)(args...);
	}

	void invoke(Args... args) override
	{
		(static_cast<T *>(this->obj_)->*func_)(args...);
	}

private:
	void (T::*func_)(Args...);
};

template<typename... Args>
class BoundStaticMethod : public BoundMethodArgs<Args...>
{
public:
	BoundStaticMethod(void (*func)(Args...))
		: BoundMethodArgs<Args...>(nullptr, nullptr, ConnectionTypeAuto),
		  func_(func)
	{
	}

	bool match(void (*func)(Args...)) const { return func == func_; }

	void activate(Args... args, bool deleteMethod = false) override
	{
		(*func_)(args...);
	}

	void invoke(Args...) override {}

private:
	void (*func_)(Args...);
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_BOUND_METHOD_H__ */
