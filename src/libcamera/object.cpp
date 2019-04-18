/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * object.cpp - Base object
 */

#include <libcamera/object.h>

#include <libcamera/signal.h>

/**
 * \file object.h
 * \brief Base object to support automatic signal disconnection
 */

namespace libcamera {

/**
 * \class Object
 * \brief Base object to support automatic signal disconnection
 *
 * The Object class simplifies signal/slot handling for classes implementing
 * slots. By inheriting from Object, an object is automatically disconnected
 * from all connected signals when it gets destroyed.
 *
 * \sa Signal
 */

Object::~Object()
{
	for (SignalBase *signal : signals_)
		signal->disconnect(this);
}

void Object::connect(SignalBase *signal)
{
	signals_.push_back(signal);
}

void Object::disconnect(SignalBase *signal)
{
	for (auto iter = signals_.begin(); iter != signals_.end(); ) {
		if (*iter == signal)
			iter = signals_.erase(iter);
		else
			iter++;
	}
}

}; /* namespace libcamera */
