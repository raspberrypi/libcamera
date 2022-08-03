/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * generated_serializer_test.cpp - Test generated serializer
 */

#include <algorithm>
#include <tuple>
#include <vector>

#include "test.h"

#include "test_ipa_interface.h"
#include "test_ipa_serializer.h"

using namespace std;
using namespace libcamera;

class IPAGeneratedSerializerTest : public Test
{
protected:
	int init() override
	{
		return TestPass;
	}

	int run() override
	{

#define TEST_FIELD_EQUALITY(struct1, struct2, field)		\
if (struct1.field != struct2.field) {				\
	cerr << #field << " field incorrect: expected \""	\
	     << t.field << "\", got \"" << u.field << "\"" << endl;\
	return TestFail;					\
}

#define TEST_SCOPED_ENUM_EQUALITY(struct1, struct2, field)	\
if (struct1.field != struct2.field) {				\
	cerr << #field << " field incorrect" << endl;		\
	return TestFail;					\
}


		ipa::test::TestStruct t, u;

		t.m = {
			{ "a", "z" },
			{ "b", "z" },
			{ "c", "z" },
			{ "d", "z" },
			{ "e", "z" },
		};

		t.a = { "a", "b", "c", "d", "e" };

		t.s1 = "hello world";
		t.s2 = "goodbye";
		t.s3 = "lorem ipsum";
		t.i  = 58527;
		t.c = ipa::test::IPAOperationInit;
		t.e = ipa::test::ErrorFlags::Error1;

		Flags<ipa::test::ErrorFlags> flags;
		flags |= ipa::test::ErrorFlags::Error1;
		flags |= ipa::test::ErrorFlags::Error2;
		t.f = flags;

		std::vector<uint8_t> serialized;

		std::tie(serialized, ignore) =
			IPADataSerializer<ipa::test::TestStruct>::serialize(t);

		u = IPADataSerializer<ipa::test::TestStruct>::deserialize(serialized);

		if (!equals(t.m, u.m))
			return TestFail;

		if (!equals(t.a, u.a))
			return TestFail;

		TEST_FIELD_EQUALITY(t, u, s1);
		TEST_FIELD_EQUALITY(t, u, s2);
		TEST_FIELD_EQUALITY(t, u, s3);
		TEST_FIELD_EQUALITY(t, u, i);
		TEST_FIELD_EQUALITY(t, u, c);

		TEST_SCOPED_ENUM_EQUALITY(t, u, e);
		TEST_SCOPED_ENUM_EQUALITY(t, u, f);

		/* Test vector of generated structs */
		std::vector<ipa::test::TestStruct> v = { t, u };
		std::vector<ipa::test::TestStruct> w;

		std::tie(serialized, ignore) =
			IPADataSerializer<vector<ipa::test::TestStruct>>::serialize(v);

		w = IPADataSerializer<vector<ipa::test::TestStruct>>::deserialize(serialized);

		if (!equals(v[0].m, w[0].m) ||
		    !equals(v[1].m, w[1].m))
			return TestFail;

		if (!equals(v[0].a, w[0].a) ||
		    !equals(v[1].a, w[1].a))
			return TestFail;

		TEST_FIELD_EQUALITY(v[0], w[0], s1);
		TEST_FIELD_EQUALITY(v[0], w[0], s2);
		TEST_FIELD_EQUALITY(v[0], w[0], s3);
		TEST_FIELD_EQUALITY(v[0], w[0], i);
		TEST_FIELD_EQUALITY(v[0], w[0], c);

		TEST_SCOPED_ENUM_EQUALITY(v[0], w[0], e);
		TEST_SCOPED_ENUM_EQUALITY(v[0], w[0], f);

		TEST_FIELD_EQUALITY(v[1], w[1], s1);
		TEST_FIELD_EQUALITY(v[1], w[1], s2);
		TEST_FIELD_EQUALITY(v[1], w[1], s3);
		TEST_FIELD_EQUALITY(v[1], w[1], i);
		TEST_FIELD_EQUALITY(v[1], w[1], c);

		TEST_SCOPED_ENUM_EQUALITY(v[1], w[1], e);
		TEST_SCOPED_ENUM_EQUALITY(v[1], w[1], f);

		return TestPass;
	}

private:
	bool equals(const map<string, string> &lhs, const map<string, string> &rhs)
	{
		bool eq = lhs.size() == rhs.size() &&
			  equal(lhs.begin(), lhs.end(), rhs.begin(),
				[](auto &a, auto &b) { return a.first == b.first &&
							      a.second == b.second; });

		if (eq)
			return true;

		cerr << "lhs:" << endl;
		for (const auto &pair : lhs)
			cerr << "- " << pair.first << ": "
			     << pair.second << endl;

		cerr << "rhs:" << endl;
		for (const auto &pair : rhs)
			cerr << "- " << pair.first << ": "
			     << pair.second << endl;

		return false;
	}

	bool equals(const vector<string> &lhs, const vector<string> &rhs)
	{
		bool eq = lhs.size() == rhs.size();

		if (!eq) {
			cerr << "sizes not equal" << endl;
			return false;
		}

		for (unsigned int i = 0; i < lhs.size(); i++)
			if (lhs[i] != rhs[i])
				eq = false;

		if (eq)
			return true;

		cerr << "lhs:" << endl;
		for (const auto &str : lhs)
			cerr << "- " << str << endl;

		cerr << "rhs:" << endl;
		for (const auto &str : rhs)
			cerr << "- " << str << endl;

		return false;
	}
};

TEST_REGISTER(IPAGeneratedSerializerTest)
