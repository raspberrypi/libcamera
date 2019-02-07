/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * signal.h - Signal & slot implementation
 */
#ifndef __LIBCAMERA_SIGNAL_H__
#define __LIBCAMERA_SIGNAL_H__

#include <list>
#include <vector>

#include <libcamera/object.h>

namespace libcamera {

template<typename... Args>
class Signal;

class SlotBase
{
public:
	SlotBase(void *obj, bool isObject)
		: obj_(obj), isObject_(isObject) {}
	virtual ~SlotBase() {}

	void *obj() { return obj_; }
	bool isObject() const { return isObject_; }

protected:
	void *obj_;
	bool isObject_;
};

template<typename... Args>
class SlotArgs : public SlotBase
{
public:
	SlotArgs(void *obj, bool isObject)
		: SlotBase(obj, isObject) {}

	virtual void invoke(Args... args) = 0;

protected:
	friend class Signal<Args...>;
};

template<typename T, typename... Args>
class SlotMember : public SlotArgs<Args...>
{
public:
	SlotMember(T *obj, bool isObject, void (T::*func)(Args...))
		: SlotArgs<Args...>(obj, isObject), func_(func) {}

	void invoke(Args... args) { (static_cast<T *>(this->obj_)->*func_)(args...); }

private:
	friend class Signal<Args...>;
	void (T::*func_)(Args...);
};

template<typename... Args>
class SlotStatic : public SlotArgs<Args...>
{
public:
	SlotStatic(void (*func)(Args...))
		: SlotArgs<Args...>(nullptr, false), func_(func) {}

	void invoke(Args... args) { (*func_)(args...); }

private:
	friend class Signal<Args...>;
	void (*func_)(Args...);
};

class SignalBase
{
public:
	template<typename T>
	void disconnect(T *object)
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotBase *slot = *iter;
			if (slot->obj() == object) {
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
			if (slot->isObject())
				static_cast<Object *>(slot->obj())->disconnect(this);
			delete slot;
		}
	}

#ifndef __DOXYGEN__
	template<typename T, typename std::enable_if<std::is_base_of<Object, T>::value>::type * = nullptr>
	void connect(T *object, void (T::*func)(Args...))
	{
		object->connect(this);
		slots_.push_back(new SlotMember<T, Args...>(object, true, func));
	}

	template<typename T, typename std::enable_if<!std::is_base_of<Object, T>::value>::type * = nullptr>
#else
	template<typename T>
#endif
	void connect(T *object, void (T::*func)(Args...))
	{
		slots_.push_back(new SlotMember<T, Args...>(object, false, func));
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
	void disconnect(T *object)
	{
		SignalBase::disconnect(object);
	}

	template<typename T>
	void disconnect(T *object, void (T::*func)(Args...))
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotArgs<Args...> *slot = static_cast<SlotArgs<Args...> *>(*iter);
			/*
			 * If the obj() pointer matches the object, the slot is
			 * guaranteed to be a member slot, so we can safely
			 * cast it to SlotMember<T, Args...> and access its
			 * func_ member.
			 */
			if (slot->obj() == object &&
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
			if (slot->obj() == nullptr &&
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
		for (SlotBase *slot : slots) {
			static_cast<SlotArgs<Args...> *>(slot)->invoke(args...);
		}
	}
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_SIGNAL_H__ */
