/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * compiler.h - Compiler support
 */
#ifndef __LIBCAMERA_COMPILER_H__
#define __LIBCAMERA_COMPILER_H__

#if __cplusplus >= 201703L
#define __nodiscard		[[nodiscard]]
#else
#define __nodiscard
#endif

#endif /* __LIBCAMERA_COMPILER_H__ */
