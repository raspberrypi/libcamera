/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2023, Ideas On Board Oy
 *
 * Transform and Orientation tests
 */

#include <iostream>

#include <libcamera/orientation.h>
#include <libcamera/transform.h>

#include "test.h"

using namespace std;
using namespace libcamera;

class TransformTest : public Test
{
protected:
	int run();
};

int TransformTest::run()
{
	/*
	 * RotationTestEntry collects two Orientation and one Transform that
	 * gets combined to validate that (o1 / o2 = T) and (o1 = o2 * T)
	 *
	 * o1 / o2 = t computes the Transform to apply to o2 to obtain o1
	 * o2 * t = o1 combines o2 with t by applying o2 first then t
	 *
	 * The comments on the (most complex) transform show how applying to
	 * an image with orientation o2 the Transform t allows to obtain o1.
	 *
	 * The image with basic rotation0 is assumed to be:
	 *
	 * 	AB
	 * 	CD
	 *
	 * And the Transform operators are:
	 *
	 * 	V = vertical flip
	 * 	H = horizontal flip
	 * 	T = transpose
	 *
	 * the operator '* (T|V)' applies V first then T.
	 */
	static const struct RotationTestEntry {
		Orientation o1;
		Orientation o2;
		Transform t;
	} testEntries[] = {
		/* Test identities transforms first. */
		{
			Orientation::Rotate0, Orientation::Rotate0,
			Transform::Identity,
		},
		{
			Orientation::Rotate0Mirror, Orientation::Rotate0Mirror,
			Transform::Identity,
		},
		{
			Orientation::Rotate180, Orientation::Rotate180,
			Transform::Identity,
		},
		{
			Orientation::Rotate180Mirror, Orientation::Rotate180Mirror,
			Transform::Identity,
		},
		{
			Orientation::Rotate90, Orientation::Rotate90,
			Transform::Identity,
		},
		{
			Orientation::Rotate90Mirror, Orientation::Rotate90Mirror,
			Transform::Identity,
		},
		{
			Orientation::Rotate270, Orientation::Rotate270,
			Transform::Identity,
		},
		{
			Orientation::Rotate270Mirror, Orientation::Rotate270Mirror,
			Transform::Identity,
		},
		/*
		 * Combine 0 and 180 degrees rotation as they're the most common
		 * ones.
		 */
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	CD  * (H|V) =  	BA	AB
			 *	BA		CD	CD
			 */
			Orientation::Rotate0, Orientation::Rotate180,
			Transform::Rot180,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	AB  * (H|V) =  	CD	DC
			 *	CD		AB	BA
			 */
			Orientation::Rotate180, Orientation::Rotate0,
			Transform::Rot180
		},
		/* Test that transpositions are handled correctly. */
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	AB  * (T|V) =  	CD	CA
			 *	CD		AB	DB
			 */
			Orientation::Rotate90, Orientation::Rotate0,
			Transform::Rot90,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	CA  * (T|H) =  	AC	AB
			 *	DB		BD	CD
			 */
			Orientation::Rotate0, Orientation::Rotate90,
			Transform::Rot270,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	AB  * (T|H) =  	BA	BD
			 *	CD		DC	AC
			 */
			Orientation::Rotate270, Orientation::Rotate0,
			Transform::Rot270,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	BD  * (T|V) =  	AC	AB
			 *	AC		BD	CD
			 */
			Orientation::Rotate0, Orientation::Rotate270,
			Transform::Rot90,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	CD  * (T|H) =  	DC	DA
			 *	BA		AB	CB
			 */
			Orientation::Rotate90, Orientation::Rotate180,
			Transform::Rot270,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	DA  * (T|V) =  	CB	CD
			 *	CB		DA	BA
			 */
			Orientation::Rotate180, Orientation::Rotate90,
			Transform::Rot90,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	CD  * (T|V) =  	BA	BC
			 *	BA		CD	AD
			 */
			Orientation::Rotate270, Orientation::Rotate180,
			Transform::Rot90,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	BC  * (T|H) =  	CB	CD
			 *	AD		DA	BA
			 */
			Orientation::Rotate180, Orientation::Rotate270,
			Transform::Rot270,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	DA  * (V|H) =  	AD	BC
			 *	CB		BC	AD
			 */
			Orientation::Rotate270, Orientation::Rotate90,
			Transform::Rot180,
		},
		/* Test that mirroring is handled correctly. */
		{
			Orientation::Rotate0, Orientation::Rotate0Mirror,
			Transform::HFlip
		},
		{
			Orientation::Rotate0Mirror, Orientation::Rotate0,
			Transform::HFlip
		},
		{
			Orientation::Rotate180, Orientation::Rotate180Mirror,
			Transform::HFlip
		},
		{
			Orientation::Rotate180Mirror, Orientation::Rotate180,
			Transform::HFlip
		},
		{
			Orientation::Rotate90, Orientation::Rotate90Mirror,
			Transform::HFlip
		},
		{
			Orientation::Rotate90Mirror, Orientation::Rotate90,
			Transform::HFlip
		},
		{
			Orientation::Rotate270, Orientation::Rotate270Mirror,
			Transform::HFlip
		},
		{
			Orientation::Rotate270Mirror, Orientation::Rotate270,
			Transform::HFlip
		},
		{
			Orientation::Rotate0, Orientation::Rotate0Mirror,
			Transform::HFlip
		},
		/*
		 * More exotic transforms which include Transpositions and
		 * mirroring.
		 */
		{
			/*
			 *      o2      t       o1
			 *      ------------------
			 *	BC  * (V) =  	AD
			 *	AD		BC
			 */
			Orientation::Rotate90Mirror, Orientation::Rotate270,
			Transform::VFlip,
		},
		{
			/*
			 *      o2      t       o1
			 *      ------------------
			 *	CB  * (T) =  	CD
			 *	DA		BA
			 */
			Orientation::Rotate180, Orientation::Rotate270Mirror,
			Transform::Transpose,
		},
		{
			/*
			 *      o2      t       o1
			 *      ------------------
			 *	AD  * (T) =  	AB
			 *	BC		DC
			 */
			Orientation::Rotate0, Orientation::Rotate90Mirror,
			Transform::Transpose,
		},
		{
			/*
			 *      o2      t       o1
			 *      ------------------
			 *	AD  * (V) =  	BC
			 *	BC		AD
			 */
			Orientation::Rotate270, Orientation::Rotate90Mirror,
			Transform::VFlip,
		},
		{
			/*
			 *      o2      t       o1
			 *      ------------------
			 *	DA  * (V) =  	CB
			 *	CB		DA
			 */
			Orientation::Rotate270Mirror, Orientation::Rotate90,
			Transform::VFlip,
		},
		{
			/*
			 *      o2      t               o1
			 *      --------------------------
			 *	CB  * (V|H) =	BC  	AD
			 *	DA		AD	BC
			 */
			Orientation::Rotate90Mirror, Orientation::Rotate270Mirror,
			Transform::Rot180,
		},
	};

	for (const auto &entry : testEntries) {
		Transform transform = entry.o1 / entry.o2;
		if (transform != entry.t) {
			cerr << "Failed to validate: " << entry.o1
			     << " / " << entry.o2
			     << " = " << transformToString(entry.t) << endl;
			cerr << "Got back: "
			     << transformToString(transform) << endl;
			return TestFail;
		}

		Orientation adjusted = entry.o2 * entry.t;
		if (adjusted != entry.o1) {
			cerr << "Failed to validate: " << entry.o2
			     << " * " << transformToString(entry.t)
			     << " = " << entry.o1 << endl;
			cerr << "Got back: " << adjusted << endl;
			return TestFail;
		}
	}

	return TestPass;
}

TEST_REGISTER(TransformTest)
