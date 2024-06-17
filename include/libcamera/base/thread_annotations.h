/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Macro of Clang thread safety analysis
 */

#pragma once

#include <libcamera/base/private.h>

/*
 * Enable thread safety attributes only with clang.
 * The attributes can be safely erased when compiling with other compilers.
 */
#if defined(__clang__) && !defined(SWIG)
#define LIBCAMERA_TSA_ATTRIBUTE__(x) __attribute__((x))
#else
#define LIBCAMERA_TSA_ATTRIBUTE__(x) /* no-op */
#endif

/* See https://clang.llvm.org/docs/ThreadSafetyAnalysis.html for these usages. */

#define LIBCAMERA_TSA_CAPABILITY(x)			\
	LIBCAMERA_TSA_ATTRIBUTE__(capability(x))

#define LIBCAMERA_TSA_SCOPED_CAPABILITY			\
	LIBCAMERA_TSA_ATTRIBUTE__(scoped_lockable)

#define LIBCAMERA_TSA_GUARDED_BY(x)			\
	LIBCAMERA_TSA_ATTRIBUTE__(guarded_by(x))

#define LIBCAMERA_TSA_PT_GUARDED_BY(x)			\
	LIBCAMERA_TSA_ATTRIBUTE__(pt_guarded_by(x))

#define LIBCAMERA_TSA_ACQUIRED_BEFORE(...)			\
	LIBCAMERA_TSA_ATTRIBUTE__(acquired_before(__VA_ARGS__))

#define LIBCAMERA_TSA_ACQUIRED_AFTER(...)			\
	LIBCAMERA_TSA_ATTRIBUTE__(acquired_after(__VA_ARGS__))

#define LIBCAMERA_TSA_REQUIRES(...)					\
	LIBCAMERA_TSA_ATTRIBUTE__(requires_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_REQUIRES_SHARED(...)				\
	LIBCAMERA_TSA_ATTRIBUTE__(requires_shared_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_ACQUIRE(...)					\
	LIBCAMERA_TSA_ATTRIBUTE__(acquire_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_ACQUIRE_SHARED(...)				\
	LIBCAMERA_TSA_ATTRIBUTE__(acquire_shared_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_RELEASE(...)					\
	LIBCAMERA_TSA_ATTRIBUTE__(release_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_RELEASE_SHARED(...)				\
	LIBCAMERA_TSA_ATTRIBUTE__(release_shared_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_RELEASE_GENERIC(...)				\
	LIBCAMERA_TSA_ATTRIBUTE__(release_generic_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_TRY_ACQUIRE(...)					\
	LIBCAMERA_TSA_ATTRIBUTE__(try_acquire_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_TRY_ACQUIRE_SHARED(...)				\
	LIBCAMERA_TSA_ATTRIBUTE__(try_acquire_shared_capability(__VA_ARGS__))

#define LIBCAMERA_TSA_EXCLUDES(...)				\
	LIBCAMERA_TSA_ATTRIBUTE__(locks_excluded(__VA_ARGS__))

#define LIBCAMERA_TSA_ASSERT_CAPABILITY(x)		\
	LIBCAMERA_TSA_ATTRIBUTE__(assert_capability(x))

#define LIBCAMERA_TSA_ASSERT_SHARED_CAPABILITY(x)		\
	LIBCAMERA_TSA_ATTRIBUTE__(assert_shared_capability(x))

#define LIBCAMERA_TSA_RETURN_CAPABILITY(x)		\
	LIBCAMERA_TSA_ATTRIBUTE__(lock_returned(x))

#define LIBCAMERA_TSA_NO_THREAD_SAFETY_ANALYSIS			\
	LIBCAMERA_TSA_ATTRIBUTE__(no_thread_safety_analysis)
