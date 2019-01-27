/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * options.h - cam - Options parsing
 */
#ifndef __CAM_OPTIONS_H__
#define __CAM_OPTIONS_H__

#include <ctype.h>
#include <map>
#include <vector>

enum OptionArgument {
	ArgumentNone,
	ArgumentRequired,
	ArgumentOptional,
};

struct Option {
	int opt;
	const char *name;
	OptionArgument argument;
	const char *argumentName;
	const char *help;

	bool hasShortOption() const { return isalnum(opt); }
	bool hasLongOption() const { return name != nullptr; }
};

template <typename T>
class OptionsBase
{
public:
	bool valid() const;
	bool isSet(const T &opt) const;
	const std::string &operator[](const T &opt) const;

private:
	friend class OptionsParser;
	std::map<T, std::string> values_;
	void clear();
};

class OptionsParser
{
public:
	class Options : public OptionsBase<int>
	{
	};

	void addOption(int opt, const char *help, const char *name = nullptr,
		       OptionArgument argument = ArgumentNone,
		       const char *argumentName = nullptr);

	Options parse(int argc, char *argv[]);
	void usage();

private:
	std::vector<Option> options_;
	std::map<unsigned int, Option *> optionsMap_;
};

#endif /* __CAM_OPTIONS_H__ */
