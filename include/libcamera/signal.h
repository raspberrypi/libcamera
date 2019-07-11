/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * signal.h - Signal & slot implementation
 */
#ifndef __LIBCAMERA_SIGNAL_H__
#define __LIBCAMERA_SIGNAL_H__

#include <list>
#include <tuple>
#include <type_traits>
#include <vector>

#include <libcamera/object.h>

namespace libcamera {

template<typename... Args>
class Signal;
class SignalBase;

class SlotBase
{
public:
	SlotBase(void *obj, Object *object)
		: obj_(obj), object_(object) {}
	virtual ~SlotBase() {}

	template<typename T, typename std::enable_if<!std::is_same<Object, T>::value>::type * = nullptr>
	bool match(T *obj) { return obj == obj_; }
	bool match(Object *object) { return object == object_; }

	void disconnect(SignalBase *signal);

	void activatePack(void *pack);
	virtual void invokePack(void *pack) = 0;

protected:
	void *obj_;
	Object *object_;
};

template<typename... Args>
class SlotArgs : public SlotBase
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
	SlotArgs(void *obj, Object *object)
		: SlotBase(obj, object) {}

	void invokePack(void *pack) override
	{
		invokePack(pack, typename generator<sizeof...(Args)>::type());
	}

	virtual void activate(Args... args) = 0;
	virtual void invoke(Args... args) = 0;
};

template<typename T, typename... Args>
class SlotMember : public SlotArgs<Args...>
{
public:
	using PackType = std::tuple<typename std::remove_reference<Args>::type...>;

	SlotMember(T *obj, Object *object, void (T::*func)(Args...))
		: SlotArgs<Args...>(obj, object), func_(func) {}

	void activate(Args... args)
	{
		if (this->object_)
			SlotBase::activatePack(new PackType{ args... });
		else
			(static_cast<T *>(this->obj_)->*func_)(args...);
	}

	void invoke(Args... args)
	{
		(static_cast<T *>(this->obj_)->*func_)(args...);
	}

private:
	friend class Signal<Args...>;
	void (T::*func_)(Args...);
};

template<typename... Args>
class SlotStatic : public SlotArgs<Args...>
{
public:
	SlotStatic(void (*func)(Args...))
		: SlotArgs<Args...>(nullptr, nullptr), func_(func) {}

	void activate(Args... args) { (*func_)(args...); }
	void invoke(Args... args) {}

private:
	friend class Signal<Args...>;
	void (*func_)(Args...);
};

class SignalBase
{
public:
	template<typename T>
	void disconnect(T *obj)
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotBase *slot = *iter;
			if (slot->match(obj)) {
				iter = slots_.erase(iter);
				delete slot;
			} else {
				++iter;
			}
		}
	}

protected:
	friend class Object;
	std::list<SlotBase *> slots_;
};

template<typename... Args>
class Signal : public SignalBase
{
public:
	Signal() {}
	~Signal()
	{
		for (SlotBase *slot : slots_) {
			slot->disconnect(this);
			delete slot;
		}
	}

#ifndef __DOXYGEN__
	template<typename T, typename std::enable_if<std::is_base_of<Object, T>::value>::type * = nullptr>
	void connect(T *obj, void (T::*func)(Args...))
	{
		Object *object = static_cast<Object *>(obj);
		object->connect(this);
		slots_.push_back(new SlotMember<T, Args...>(obj, object, func));
	}

	template<typename T, typename std::enable_if<!std::is_base_of<Object, T>::value>::type * = nullptr>
#else
	template<typename T>
#endif
	void connect(T *obj, void (T::*func)(Args...))
	{
		slots_.push_back(new SlotMember<T, Args...>(obj, nullptr, func));
	}

	void connect(void (*func)(Args...))
	{
		slots_.push_back(new SlotStatic<Args...>(func));
	}

	void disconnect()
	{
		for (SlotBase *slot : slots_)
			delete slot;
		slots_.clear();
	}

	template<typename T>
	void disconnect(T *obj)
	{
		SignalBase::disconnect(obj);
	}

	template<typename T>
	void disconnect(T *obj, void (T::*func)(Args...))
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotArgs<Args...> *slot = static_cast<SlotArgs<Args...> *>(*iter);
			/*
			 * If the object matches the slot, the slot is
			 * guaranteed to be a member slot, so we can safely
			 * cast it to SlotMember<T, Args...> and access its
			 * func_ member.
			 */
			if (slot->match(obj) &&
			    static_cast<SlotMember<T, Args...> *>(slot)->func_ == func) {
				iter = slots_.erase(iter);
				delete slot;
			} else {
				++iter;
			}
		}
	}

	void disconnect(void (*func)(Args...))
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotArgs<Args...> *slot = *iter;
			if (slot->match(nullptr) &&
			    static_cast<SlotStatic<Args...> *>(slot)->func_ == func) {
				iter = slots_.erase(iter);
				delete slot;
			} else {
				++iter;
			}
		}
	}

	void emit(Args... args)
	{
		/*
		 * Make a copy of the slots list as the slot could call the
		 * disconnect operation, invalidating the iterator.
		 */
		std::vector<SlotBase *> slots{ slots_.begin(), slots_.end() };
		for (SlotBase *slot : slots)
			static_cast<SlotArgs<Args...> *>(slot)->activate(args...);
	}
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_SIGNAL_H__ */
