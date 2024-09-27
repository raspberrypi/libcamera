/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024 Red Hat, Inc.
 *
 * Simple pipeline IPA Context
 */

#pragma once

#include <libipa/fc_queue.h>

namespace libcamera {

namespace ipa::soft {

struct IPASessionConfiguration {
};

struct IPAActiveState {
};

struct IPAFrameContext : public FrameContext {
};

struct IPAContext {
	IPASessionConfiguration configuration;
	IPAActiveState activeState;
	FCQueue<IPAFrameContext> frameContexts;
};

} /* namespace ipa::soft */

} /* namespace libcamera */
