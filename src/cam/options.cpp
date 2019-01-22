/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * options.cpp - cam - Options parsing
 */

#include <getopt.h>
#include <iomanip>
#include <iostream>
#include <string.h>

#include "options.h"

void OptionsParser::addOption(int opt, const char *help, const char *name,
			      OptionArgument argument, const char *argumentName)
{
	/*
	 * Options must have at least a short or long name, and a text message.
	 * If an argument is accepted, it must be described by argumentName.
	 */
	if (!isalnum(opt) && !name)
		return;
	if (!help || help[0] == '\0')
		return;
	if (argument != ArgumentNone && !argumentName)
		return;

	/* Reject duplicate options. */
	if (optionsMap_.find(opt) != optionsMap_.end())
		return;

	options_.push_back(Option({ opt, name, argument, argumentName, help }));
	optionsMap_[opt] = &options_.back();
}

OptionsParser::Options OptionsParser::parse(int argc, char **argv)
{
	OptionsParser::Options options;

	/*
	 * Allocate short and long options arrays large enough to contain all
	 * options.
	 */
	char shortOptions[options_.size() * 3 + 2] = {};
	struct option longOptions[options_.size() + 1] = {};
	unsigned int ids = 0;
	unsigned int idl = 0;

	shortOptions[ids++] = ':';

	for (const Option &option : options_) {
		if (option.hasShortOption()) {
			shortOptions[ids++] = option.opt;
			if (option.argument != ArgumentNone)
				shortOptions[ids++] = ':';
			if (option.argument == ArgumentOptional)
				shortOptions[ids++] = ':';
		}

		if (option.hasLongOption()) {
			longOptions[idl].name = option.name;

			switch (option.argument) {
			case ArgumentNone:
				longOptions[idl].has_arg = no_argument;
				break;
			case ArgumentRequired:
				longOptions[idl].has_arg = required_argument;
				break;
			case ArgumentOptional:
				longOptions[idl].has_arg = optional_argument;
				break;
			}

			longOptions[idl].flag = 0;
			longOptions[idl].val = option.opt;
			idl++;
		}
	}

	opterr = 0;

	while (true) {
		int c = getopt_long(argc, argv, shortOptions, longOptions, nullptr);

		if (c == -1)
			break;

		if (c == '?' || c == ':') {
			if (c == '?')
				std::cerr << "Invalid option ";
			else
				std::cerr << "Missing argument for option ";
			std::cerr << argv[optind - 1] << std::endl;

			usage();
			options.clear();
			break;
		}

		options.values_[c] = optarg ? optarg : "";
	}

	return options;
}

void OptionsParser::usage()
{
	std::cerr << "Options:" << std::endl;

	unsigned int indent = 0;

	for (const Option &option : options_) {
		unsigned int length = 14;
		if (option.hasLongOption())
			length += 2 + strlen(option.name);
		if (option.argument != ArgumentNone)
			length += 1 + strlen(option.argumentName);
		if (option.argument == ArgumentOptional)
			length += 2;

		if (length > indent)
			indent = length;
	}

	indent = (indent + 7) / 8 * 8;

	for (const Option &option : options_) {
		std::string argument;
		if (option.hasShortOption())
			argument = std::string("  -")
				 + static_cast<char>(option.opt);
		else
			argument = "    ";

		if (option.hasLongOption()) {
			if (option.hasShortOption())
				argument += ", ";
			else
				argument += "  ";
			argument += std::string("--") + option.name;
		};

		if (option.argument != ArgumentNone) {
			argument += std::string(" ");
			if (option.argument == ArgumentOptional)
				argument += "[";
			argument += option.argumentName;
			if (option.argument == ArgumentOptional)
				argument += "]";
		}

		std::cerr << std::setw(indent) << std::left << argument;
		std::cerr << option.help << std::endl;
	}
}

OptionsParser::Options::Options()
{
}

bool OptionsParser::Options::valid() const
{
	return !values_.empty();
}

bool OptionsParser::Options::isSet(int opt) const
{
	return values_.find(opt) != values_.end();
}

const std::string &OptionsParser::Options::operator[](int opt) const
{
	return values_.find(opt)->second;
}

void OptionsParser::Options::clear()
{
	values_.clear();
}
