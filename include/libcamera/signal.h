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

namespace libcamera {

template<typename... Args>
class Signal;

template<typename... Args>
class SlotBase
{
public:
	SlotBase(void *obj)
		: obj_(obj) {}
	virtual ~SlotBase() {}

	virtual void invoke(Args... args) = 0;

protected:
	friend class Signal<Args...>;
	void *obj_;
};

template<typename T, typename... Args>
class SlotMember : public SlotBase<Args...>
{
public:
	SlotMember(T *obj, void (T::*func)(Args...))
		: SlotBase<Args...>(obj), func_(func) {}

	void invoke(Args... args) { (static_cast<T *>(this->obj_)->*func_)(args...); }

private:
	friend class Signal<Args...>;
	void (T::*func_)(Args...);
};

template<typename... Args>
class SlotStatic : public SlotBase<Args...>
{
public:
	SlotStatic(void (*func)(Args...))
		: SlotBase<Args...>(nullptr), func_(func) {}

	void invoke(Args... args) { (*func_)(args...); }

private:
	friend class Signal<Args...>;
	void (*func_)(Args...);
};

template<typename... Args>
class Signal
{
public:
	Signal() {}
	~Signal()
	{
		for (SlotBase<Args...> *slot : slots_)
			delete slot;
	}

	template<typename T>
	void connect(T *object, void (T::*func)(Args...))
	{
		slots_.push_back(new SlotMember<T, Args...>(object, func));
	}

	void connect(void (*func)(Args...))
	{
		slots_.push_back(new SlotStatic<Args...>(func));
	}

	void disconnect()
	{
		for (SlotBase<Args...> *slot : slots_)
			delete slot;
		slots_.clear();
	}

	template<typename T>
	void disconnect(T *object)
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotBase<Args...> *slot = *iter;
			if (slot->obj_ == object) {
				iter = slots_.erase(iter);
				delete slot;
			} else {
				++iter;
			}
		}
	}

	template<typename T>
	void disconnect(T *object, void (T::*func)(Args...))
	{
		for (auto iter = slots_.begin(); iter != slots_.end(); ) {
			SlotBase<Args...> *slot = *iter;
			/*
			 * If the obj_ pointer matches the object types must
			 * match, so we can safely cast to SlotMember<T, Args>.
			 */
			if (slot->obj_ == object &&
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
			SlotBase<Args...> *slot = *iter;
			if (slot->obj_ == nullptr &&
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
		std::vector<SlotBase<Args...> *> slots{ slots_.begin(), slots_.end() };
		for (SlotBase<Args...> *slot : slots)
			slot->invoke(args...);
	}

private:
	std::list<SlotBase<Args...> *> slots_;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_SIGNAL_H__ */
