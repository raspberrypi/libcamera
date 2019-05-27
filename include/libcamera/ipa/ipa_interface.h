/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_interface.h - Image Processing Algorithm interface
 */
#ifndef __LIBCAMERA_IPA_INTERFACE_H__
#define __LIBCAMERA_IPA_INTERFACE_H__

namespace libcamera {

class IPAInterface
{
public:
	virtual ~IPAInterface() {}

	virtual int init() = 0;
};

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_INTERFACE_H__ */
