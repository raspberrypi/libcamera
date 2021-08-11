/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Ideas On Board
 *
 * algorithm.h - IPU3 control algorithm interface
 */
#ifndef __LIBCAMERA_IPA_IPU3_ALGORITHM_H__
#define __LIBCAMERA_IPA_IPU3_ALGORITHM_H__

namespace libcamera {

namespace ipa::ipu3 {

class Algorithm
{
public:
	virtual ~Algorithm() {}
};

} /* namespace ipa::ipu3 */

} /* namespace libcamera */

#endif /* __LIBCAMERA_IPA_IPU3_ALGORITHM_H__ */
