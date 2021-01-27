/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * results.cpp - Test result aggregator
 */

#include "results.h"

#include <iostream>

void Results::add(const Result &result)
{
	if (result.first == Pass)
		passed_++;
	else if (result.first == Fail)
		failed_++;
	else if (result.first == Skip)
		skipped_++;

	printResult(result);
}

void Results::add(Status status, const std::string &message)
{
	add({ status, message });
}

void Results::fail(const std::string &message)
{
	add(Fail, message);
}

void Results::pass(const std::string &message)
{
	add(Pass, message);
}

void Results::skip(const std::string &message)
{
	add(Skip, message);
}

int Results::summary() const
{
	if (failed_ + passed_ + skipped_ != planned_) {
		std::cout << "Planned and executed number of tests differ "
			  << failed_ + passed_ + skipped_ << " executed "
			  << planned_ << " planned" << std::endl;

		return -EINVAL;
	}

	std::cout << planned_ << " tests executed, "
		  << passed_ << " tests passed, "
		  << skipped_ << " tests skipped and "
		  << failed_ << " tests failed " << std::endl;

	return 0;
}

void Results::printResult(const Result &result)
{
	std::string prefix;

	/* \todo Make parsable as TAP. */
	if (result.first == Pass)
		prefix = "PASS";
	else if (result.first == Fail)
		prefix = "FAIL";
	else if (result.first == Skip)
		prefix = "SKIP";

	std::cout << "- " << prefix << ": " << result.second << std::endl;
}
