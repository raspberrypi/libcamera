/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * pub_key.h - Public key signature verification
 */

#pragma once

#include <stdint.h>

#include <libcamera/base/span.h>

#if HAVE_CRYPTO
struct evp_pkey_st;
#elif HAVE_GNUTLS
struct gnutls_pubkey_st;
#endif

namespace libcamera {

class PubKey
{
public:
	PubKey(Span<const uint8_t> key);
	~PubKey();

	bool isValid() const { return valid_; }
	bool verify(Span<const uint8_t> data, Span<const uint8_t> sig) const;

private:
	bool valid_;
#if HAVE_CRYPTO
	struct evp_pkey_st *pubkey_;
#elif HAVE_GNUTLS
	struct gnutls_pubkey_st *pubkey_;
#endif
};

} /* namespace libcamera */
