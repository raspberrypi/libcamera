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
	BoundMethodBase(void *obj, Object *object)
		: obj_(obj), object_(object) {}
	virtual ~BoundMethodBase() {}

	template<typename T, typename std::enable_if<!std::is_same<Object, T>::value>::type * = nullptr>
	bool match(T *obj) { return obj == obj_; }
	bool match(Object *object) { return object == object_; }

	Object *object() const { return object_; }

	void activatePack(void *pack);
	virtual void invokePack(void *pack) = 0;

protected:
	void *obj_;
	Object *object_;
};

template<typename... Args>
class BoundMethodArgs : public BoundMethodBase
{
private:
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

	using PackType = std::tuple<typename std::remove_reference<Args>::type...>;

	template<int... S>
	void invokePack(void *pack, sequence<S...>)
	{
		PackType *args = static_cast<PackType *>(pack);
		invoke(std::get<S>(*args)...);
		delete args;
	}

public:
	BoundMethodArgs(void *obj, Object *object)
		: BoundMethodBase(obj, object) {}

	void invokePack(void *pack) override
	{
		invokePack(pack, typename generator<sizeof...(Args)>::type());
	}

	virtual void activate(Args... args) = 0;
	virtual void invoke(Args... args) = 0;
};

template<typename T, typename... Args>
class BoundMemberMethod : public BoundMethodArgs<Args...>
{
public:
	using PackType = std::tuple<typename std::remove_reference<Args>::type...>;

	BoundMemberMethod(T *obj, Object *object, void (T::*func)(Args...))
		: BoundMethodArgs<Args...>(obj, object), func_(func) {}

	bool match(void (T::*func)(Args...)) const { return func == func_; }

	void activate(Args... args)
	{
		if (this->object_)
			BoundMethodBase::activatePack(new PackType{ args... });
		else
			(static_cast<T *>(this->obj_)->*func_)(args...);
	}

	void invoke(Args... args)
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
		: BoundMethodArgs<Args...>(nullptr, nullptr), func_(func) {}

	bool match(void (*func)(Args...)) const { return func == func_; }

	void activate(Args... args) { (*func_)(args...); }
	void invoke(Args...) {}

private:
	void (*func_)(Args...);
};

}; /* namespace libcamera */

#endif /* __LIBCAMERA_BOUND_METHOD_H__ */
