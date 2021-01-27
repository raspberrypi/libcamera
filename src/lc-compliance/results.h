/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * results.h - Test result aggregator
 */
#ifndef __LC_COMPLIANCE_RESULTS_H__
#define __LC_COMPLIANCE_RESULTS_H__

#include <string>
#include <utility>

/* \todo Check if result aggregator can be shared with self tests in test/ */
class Results
{
public:
	enum Status {
		Fail,
		Pass,
		Skip,
	};

	using Result = std::pair<Status, std::string>;

	Results(unsigned int planned)
		: planned_(planned), passed_(0), failed_(0), skipped_(0)
	{
	}

	void add(const Result &result);
	void add(Status status, const std::string &message);
	void fail(const std::string &message);
	void pass(const std::string &message);
	void skip(const std::string &message);

	int summary() const;

private:
	void printResult(const Result &result);

	unsigned int planned_;
	unsigned int passed_;
	unsigned int failed_;
	unsigned int skipped_;
};

#endif /* __LC_COMPLIANCE_RESULTS_H__ */
