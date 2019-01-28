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

class OptionsParser
{
public:
	class Options {
	public:
		Options();

		bool valid() const;
		bool isSet(int opt) const;
		const std::string &operator[](int opt) const;

	private:
		friend class OptionsParser;
		std::map<int, std::string> values_;
		void clear();
	};

	void addOption(int opt, const char *help, const char *name = nullptr,
		       OptionArgument argument = ArgumentNone,
		       const char *argumentName = nullptr);

	Options parse(int argc, char *argv[]);
	void usage();

private:
	struct Option {
		int opt;
		const char *name;
		OptionArgument argument;
		const char *argumentName;
		const char *help;

		bool hasShortOption() const { return isalnum(opt); }
		bool hasLongOption() const { return name != nullptr; }
	};

	std::vector<Option> options_;
	std::map<unsigned int, Option *> optionsMap_;
};

#endif /* __CAM_OPTIONS_H__ */
