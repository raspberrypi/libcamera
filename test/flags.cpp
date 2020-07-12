/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * flags.cpp - Flags tests
 */

#include <iostream>

#include <libcamera/base/flags.h>

#include "test.h"

using namespace libcamera;
using namespace std;

class FlagsTest : public Test
{
protected:
	enum class Option {
		First = (1 << 0),
		Second = (1 << 1),
		Third = (1 << 2),
	};

	using Options = Flags<Option>;

	enum class Mode {
		Alpha = (1 << 0),
		Beta = (1 << 1),
		Gamma = (1 << 2),
	};

	using Modes = Flags<Mode>;

	int run() override;
};

namespace libcamera {

LIBCAMERA_FLAGS_ENABLE_OPERATORS(FlagsTest::Option)

} /* namespace libcamera */

int FlagsTest::run()
{
	/* Commented-out constructs are expected not to compile. */

	/* Flags<int> f; */

	/*
	 * Unary operators with enum argument.
	 */

	Options options;

	if (options) {
		cerr << "Default-constructed Flags<> is not zero" << endl;
		return TestFail;
	}

	options |= Option::First;
	if (!options || options != Option::First) {
		cerr << "Unary bitwise OR with enum failed" << endl;
		return TestFail;
	}

	/* options &= Mode::Alpha; */
	/* options |= Mode::Beta;  */
	/* options ^= Mode::Gamma; */

	options &= ~Option::First;
	if (options) {
		cerr << "Unary bitwise AND with enum failed" << endl;
		return TestFail;
	}

	options ^= Option::Second;
	if (options != Option::Second) {
		cerr << "Unary bitwise XOR with enum failed" << endl;
		return TestFail;
	}

	options = Options();

	/*
	 * Unary operators with Flags argument.
	 */

	options |= Options(Option::First);
	if (options != Option::First) {
		cerr << "Unary bitwise OR with Flags<> failed" << endl;
		return TestFail;
	}

	/* options &= Options(Mode::Alpha); */
	/* options |= Options(Mode::Beta);  */
	/* options ^= Options(Mode::Gamma); */

	options &= ~Options(Option::First);
	if (options) {
		cerr << "Unary bitwise AND with Flags<> failed" << endl;
		return TestFail;
	}

	options ^= Options(Option::Second);
	if (options != Option::Second) {
		cerr << "Unary bitwise XOR with Flags<> failed" << endl;
		return TestFail;
	}

	options = Options();

	/*
	 * Binary operators with enum argument.
	 */

	options = options | Option::First;
	if (!(options & Option::First)) {
		cerr << "Binary bitwise OR with enum failed" << endl;
		return TestFail;
	}

	/* options = options & Mode::Alpha; */
	/* options = options | Mode::Beta;  */
	/* options = options ^ Mode::Gamma; */

	options = options & ~Option::First;
	if (options != (Option::First & Option::Second)) {
		cerr << "Binary bitwise AND with enum failed" << endl;
		return TestFail;
	}

	options = options ^ (Option::First ^ Option::Second);
	if (options != (Option::First | Option::Second)) {
		cerr << "Binary bitwise XOR with enum failed" << endl;
		return TestFail;
	}

	options = Options();

	/*
	 * Binary operators with Flags argument.
	 */

	options |= Options(Option::First);
	if (!(options & Option::First)) {
		cerr << "Binary bitwise OR with Flags<> failed" << endl;
		return TestFail;
	}

	/* options = options & Options(Mode::Alpha); */
	/* options = options | Options(Mode::Beta);  */
	/* options = options ^ Options(Mode::Gamma); */

	options = options & ~Options(Option::First);
	if (options) {
		cerr << "Binary bitwise AND with Flags<> failed" << endl;
		return TestFail;
	}

	options = options ^ Options(Option::Second);
	if (options != Option::Second) {
		cerr << "Binary bitwise XOR with Flags<> failed" << endl;
		return TestFail;
	}

	options = Options();

	/*
	 * Conversion operators.
	 */

	options |= Option::First | Option::Second | Option::Third;
	if (static_cast<Options::Type>(options) != 7) {
		cerr << "Cast to underlying type failed" << endl;
		return TestFail;
	}

	/*
	 * Conversion of the result of ninary operators on the underlying enum.
	 */

	/* unsigned int val1 = Option::First; */
	/* unsigned int val2 = ~Option::First; */
	/* unsigned int val3 = Option::First | Option::Second; */
	/* Option val4 = ~Option::First; */
	/* Option val5 = Option::First | Option::Second; */

	return TestPass;
}

TEST_REGISTER(FlagsTest)
