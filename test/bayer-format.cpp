/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Sebastian Fricke
 *
 * bayer_format.cpp - BayerFormat class tests
 */

#include <iostream>

#include <libcamera/transform.h>

#include "libcamera/internal/bayer_format.h"

#include "test.h"

using namespace std;
using namespace libcamera;

class BayerFormatTest : public Test
{
protected:
	int run()
	{
		/* An empty Bayer format has to be invalid. */
		BayerFormat bayerFmt;
		if (bayerFmt.isValid()) {
			cerr << "An empty BayerFormat has to be invalid."
			     << endl;
			return TestFail;
		}

		/* A correct Bayer format has to be valid. */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		if (!bayerFmt.isValid()) {
			cerr << "A correct BayerFormat has to be valid."
			     << endl;
			return TestFail;
		}

		/*
		 * Two bayer formats created with the same order and bit depth
		 * have to be equal.
		 */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		BayerFormat bayerFmtExpect = BayerFormat(BayerFormat::BGGR, 8,
							 BayerFormat::Packing::None);
		if (bayerFmt != bayerFmtExpect) {
			cerr << "Comparison of identical formats failed."
			     << endl;
			return TestFail;
		}

		/*
		 * Two Bayer formats created with the same order but with a
		 * different bitDepth are not equal.
		 */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		bayerFmtExpect = BayerFormat(BayerFormat::BGGR, 12,
					     BayerFormat::Packing::None);
		if (bayerFmt == bayerFmtExpect) {
			cerr << "Comparison of different formats failed."
			     << endl;
			return TestFail;
		}

		/*
		 * Create a Bayer format with a V4L2PixelFormat and check if we
		 * get the same format after converting back to the V4L2 Format.
		 */
		V4L2PixelFormat v4l2FmtExpect = V4L2PixelFormat(
			V4L2_PIX_FMT_SBGGR8);
		bayerFmt = BayerFormat::fromV4L2PixelFormat(v4l2FmtExpect);
		V4L2PixelFormat v4l2Fmt = bayerFmt.toV4L2PixelFormat();
		if (v4l2Fmt != v4l2FmtExpect) {
			cerr << "Expected: '" << v4l2FmtExpect
			     << "' got: '" << v4l2Fmt << "'" << endl;
			return TestFail;
		}

		/*
		 * Use an empty Bayer format and verify that no matching
		 * V4L2PixelFormat is found.
		 */
		v4l2FmtExpect = V4L2PixelFormat();
		bayerFmt = BayerFormat();
		v4l2Fmt = bayerFmt.toV4L2PixelFormat();
		if (v4l2Fmt != v4l2FmtExpect) {
			cerr << "Expected: empty V4L2PixelFormat got: '"
			     << v4l2Fmt << "'" << endl;
			return TestFail;
		}

		/*
		 * Check if we get the expected Bayer format BGGR8
		 * when we convert the V4L2PixelFormat (V4L2_PIX_FMT_SBGGR8)
		 * to a Bayer format.
		 */
		bayerFmtExpect = BayerFormat(BayerFormat::BGGR, 8,
					     BayerFormat::Packing::None);
		v4l2Fmt = V4L2PixelFormat(V4L2_PIX_FMT_SBGGR8);
		bayerFmt = BayerFormat::fromV4L2PixelFormat(v4l2Fmt);
		if (bayerFmt != bayerFmtExpect) {
			cerr << "Expected BayerFormat '"
			     << bayerFmtExpect << "', got: '"
			     << bayerFmt << "'" << endl;
			return TestFail;
		}

		/*
		 * Confirm that a V4L2PixelFormat that is not found in
		 * the conversion table, doesn't yield a Bayer format.
		 */
		V4L2PixelFormat v4l2FmtUnknown = V4L2PixelFormat(
			V4L2_PIX_FMT_BGRA444);
		bayerFmt = BayerFormat::fromV4L2PixelFormat(v4l2FmtUnknown);
		if (bayerFmt.isValid()) {
			cerr << "Expected empty BayerFormat got: '"
			     << bayerFmt << "'" << endl;
			return TestFail;
		}

		/*
		 * Test if a valid Bayer format can be converted to a
		 * string representation.
		 */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		if (bayerFmt.toString() != "BGGR-8") {
			cerr << "String representation != 'BGGR-8' (got: '"
			     << bayerFmt.toString() << "' ) " << endl;
			return TestFail;
		}

		/*
		 * Determine if an empty Bayer format results in no
		 * string representation.
		 */
		bayerFmt = BayerFormat();
		if (bayerFmt.toString() != "INVALID") {
			cerr << "String representation != 'INVALID' (got: '"
			     << bayerFmt.toString() << "' ) " << endl;
			return TestFail;
		}

		/*
		 * Perform a horizontal Flip and make sure that the
		 * order is adjusted accordingly.
		 */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		bayerFmtExpect = BayerFormat(BayerFormat::GBRG, 8,
					     BayerFormat::Packing::None);
		BayerFormat hFlipFmt = bayerFmt.transform(Transform::HFlip);
		if (hFlipFmt != bayerFmtExpect) {
			cerr << "Horizontal flip of 'BGGR-8' should result in '"
			     << bayerFmtExpect << "', got: '"
			     << hFlipFmt << "'" << endl;
			return TestFail;
		}

		/*
		 * Perform a vertical Flip and make sure that
		 * the order is adjusted accordingly.
		 */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		bayerFmtExpect = BayerFormat(BayerFormat::GRBG, 8,
					     BayerFormat::Packing::None);
		BayerFormat vFlipFmt = bayerFmt.transform(Transform::VFlip);
		if (vFlipFmt != bayerFmtExpect) {
			cerr << "Vertical flip of 'BGGR-8' should result in '"
			     << bayerFmtExpect << "', got: '"
			     << vFlipFmt << "'" << endl;
			return TestFail;
		}

		/*
		 * Perform a transposition on a pixel order with both green
		 * pixels on the bottom left to top right diagonal and make
		 * sure, that it doesn't change.
		 */
		bayerFmt = BayerFormat(BayerFormat::BGGR, 8, BayerFormat::Packing::None);
		BayerFormat transposeFmt = bayerFmt.transform(
			Transform::Transpose);
		if (transposeFmt != bayerFmt) {
			cerr << "Transpose with both green pixels on the "
			     << "antidiagonal should not change the order "
			     << "(got '" << transposeFmt << "')"
			     << endl;
			return TestFail;
		}

		/*
		 * Perform a transposition on an pixel order with red and blue
		 * on the bottom left to top right diagonal and make sure
		 * that their positions are switched.
		 */
		bayerFmt = BayerFormat(BayerFormat::GBRG, 8, BayerFormat::Packing::None);
		bayerFmtExpect = BayerFormat(BayerFormat::GRBG, 8,
					     BayerFormat::Packing::None);
		transposeFmt = bayerFmt.transform(Transform::Transpose);
		if (transposeFmt != bayerFmtExpect) {
			cerr << "Transpose with the red & blue pixels on the "
			     << "antidiagonal should switch their position "
			     << "(got '" << transposeFmt << "')"
			     << endl;
			return TestFail;
		}

		return TestPass;
	}
};

TEST_REGISTER(BayerFormatTest)
