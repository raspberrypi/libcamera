/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * pub_key.cpp - Public key signature verification
 */

#include "libcamera/internal/pub_key.h"

#if HAVE_CRYPTO
#include <openssl/evp.h>
#include <openssl/rsa.h>
#include <openssl/sha.h>
#include <openssl/x509.h>
#elif HAVE_GNUTLS
#include <gnutls/abstract.h>
#endif

/**
 * \file pub_key.h
 * \brief Public key signature verification
 */

namespace libcamera {

/**
 * \class PubKey
 * \brief Public key wrapper for signature verification
 *
 * The PubKey class wraps a public key and implements signature verification. It
 * only supports RSA keys and the RSA-SHA256 signature algorithm.
 */

/**
 * \brief Construct a PubKey from key data
 * \param[in] key Key data encoded in DER format
 */
PubKey::PubKey([[maybe_unused]] Span<const uint8_t> key)
	: valid_(false)
{
#if HAVE_CRYPTO
	const uint8_t *data = key.data();
	pubkey_ = d2i_PUBKEY(nullptr, &data, key.size());
	if (!pubkey_)
		return;

	valid_ = true;
#elif HAVE_GNUTLS
	int ret = gnutls_pubkey_init(&pubkey_);
	if (ret < 0)
		return;

	const gnutls_datum_t gnuTlsKey{
		const_cast<unsigned char *>(key.data()),
		static_cast<unsigned int>(key.size())
	};
	ret = gnutls_pubkey_import(pubkey_, &gnuTlsKey, GNUTLS_X509_FMT_DER);
	if (ret < 0)
		return;

	valid_ = true;
#endif
}

PubKey::~PubKey()
{
#if HAVE_CRYPTO
	EVP_PKEY_free(pubkey_);
#elif HAVE_GNUTLS
	gnutls_pubkey_deinit(pubkey_);
#endif
}

/**
 * \fn bool PubKey::isValid() const
 * \brief Check is the public key is valid
 * \return True if the public key is valid, false otherwise
 */

/**
 * \brief Verify signature on data
 * \param[in] data The signed data
 * \param[in] sig The signature
 *
 * Verify that the signature \a sig matches the signed \a data for the public
 * key. The signture algorithm is hardcoded to RSA-SHA256.
 *
 * \return True if the signature is valid, false otherwise
 */
bool PubKey::verify([[maybe_unused]] Span<const uint8_t> data,
		    [[maybe_unused]] Span<const uint8_t> sig) const
{
	if (!valid_)
		return false;

#if HAVE_CRYPTO
	/*
	 * Create and initialize a public key algorithm context for signature
	 * verification.
	 */
	EVP_PKEY_CTX *ctx = EVP_PKEY_CTX_new(pubkey_, nullptr);
	if (!ctx)
		return false;

	if (EVP_PKEY_verify_init(ctx) <= 0 ||
	    EVP_PKEY_CTX_set_rsa_padding(ctx, RSA_PKCS1_PADDING) <= 0 ||
	    EVP_PKEY_CTX_set_signature_md(ctx, EVP_sha256()) <= 0) {
		EVP_PKEY_CTX_free(ctx);
		return false;
	}

	/* Calculate the SHA256 digest of the data. */
	uint8_t digest[SHA256_DIGEST_LENGTH];
	SHA256(data.data(), data.size(), digest);

	/* Decrypt the signature and verify it matches the digest. */
	int ret = EVP_PKEY_verify(ctx, sig.data(), sig.size(), digest,
				  SHA256_DIGEST_LENGTH);
	EVP_PKEY_CTX_free(ctx);
	return ret == 1;
#elif HAVE_GNUTLS
	const gnutls_datum_t gnuTlsData{
		const_cast<unsigned char *>(data.data()),
		static_cast<unsigned int>(data.size())
	};

	const gnutls_datum_t gnuTlsSig{
		const_cast<unsigned char *>(sig.data()),
		static_cast<unsigned int>(sig.size())
	};

	int ret = gnutls_pubkey_verify_data2(pubkey_, GNUTLS_SIGN_RSA_SHA256, 0,
					     &gnuTlsData, &gnuTlsSig);
	return ret >= 0;
#else
	return false;
#endif
}

} /* namespace libcamera */
