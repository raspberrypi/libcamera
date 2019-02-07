/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * object.h - Base object
 */
#ifndef __LIBCAMERA_OBJECT_H__
#define __LIBCAMERA_OBJECT_H__

#include <list>

namespace libcamera {

class SignalBase;
template<typename... Args>
class Signal;

class Object
{
public:
	virtual ~Object();

private:
	template<typename... Args>
	friend class Signal;

	void connect(SignalBase *signal);
	void disconnect(SignalBase *signal);

	std::list<SignalBase *> signals_;
};

}; /* namespace libcamera */

#endif /* __LIBCAMERA_OBJECT_H__ */
