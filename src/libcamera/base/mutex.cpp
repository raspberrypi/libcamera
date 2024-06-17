/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Mutex classes with clang thread safety annotation
 */

#include <libcamera/base/mutex.h>

/**
 * \file base/mutex.h
 * \brief Mutex classes with clang thread safety annotation
 */

namespace libcamera {

/**
 * \class Mutex
 * \brief std::mutex wrapper with clang thread safety annotation
 *
 * The Mutex class wraps a std::mutex instance to add clang thread safety
 * annotation support. The class exposes the same interface as std::mutex and
 * can be used as a transparent replacement. It integrates with the
 * MutexLocker and ConditionVariable classes.
 *
 * See https://en.cppreference.com/w/cpp/thread/mutex for the complete API
 * documentation.
 */

/**
 * \class MutexLocker
 * \brief std::unique_lock wrapper with clang thread safety annotation
 *
 * The MutexLocker class wraps a std::unique_lock instance to add clang thread
 * safety annotation support. The class exposes the same interface as
 * std::unique_lock and can be used as a transparent replacement. It integrates
 * with the Mutex and ConditionVariable classes.
 *
 * See https://en.cppreference.com/w/cpp/thread/unique_lock for the complete API
 * documentation.
 */

/**
 * \class ConditionVariable
 * \brief std::condition_variable wrapper integrating with MutexLocker
 *
 * The ConditionVariable class wraps a std::condition_variable instance to
 * integrate with the MutexLocker class. The class exposes the same interface as
 * std::condition_variable and can be used as a transparent replacement.
 *
 * See https://en.cppreference.com/w/cpp/thread/condition_variable for the
 * complete API documentation.
 */

} /* namespace libcamera */
