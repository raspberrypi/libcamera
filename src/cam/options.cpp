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

/* -----------------------------------------------------------------------------
 * Option
 */

const char *Option::typeName() const
{
	switch (type) {
	case OptionNone:
		return "none";

	case OptionInteger:
		return "integer";

	case OptionString:
		return "string";

	case OptionKeyValue:
		return "key=value";
	}

	return "unknown";
}

/* -----------------------------------------------------------------------------
 * OptionBase<T>
 */

template <typename T>
bool OptionsBase<T>::valid() const
{
	return !values_.empty();
}

template <typename T>
bool OptionsBase<T>::isSet(const T &opt) const
{
	return values_.find(opt) != values_.end();
}

template <typename T>
const OptionValue &OptionsBase<T>::operator[](const T &opt) const
{
	return values_.find(opt)->second;
}

template <typename T>
bool OptionsBase<T>::parseValue(const T &opt, const Option &option,
				const char *optarg)
{
	OptionValue value;

	switch (option.type) {
	case OptionNone:
		break;

	case OptionInteger:
		unsigned int integer;

		if (optarg) {
			char *endptr;
			integer = strtoul(optarg, &endptr, 10);
			if (*endptr != '\0')
				return false;
		} else {
			integer = 0;
		}

		value = OptionValue(integer);
		break;

	case OptionString:
		value = OptionValue(optarg ? optarg : "");
		break;

	case OptionKeyValue:
		KeyValueParser *kvParser = option.keyValueParser;
		KeyValueParser::Options keyValues = kvParser->parse(optarg);
		if (!keyValues.valid())
			return false;

		value = OptionValue(keyValues);
		break;
	}

	values_[opt] = value;
	return true;
}

template <typename T>
void OptionsBase<T>::clear()
{
	values_.clear();
}

template class OptionsBase<int>;
template class OptionsBase<std::string>;

/* -----------------------------------------------------------------------------
 * KeyValueParser
 */

bool KeyValueParser::addOption(const char *name, OptionType type,
			       const char *help, OptionArgument argument)
{
	if (!name)
		return false;
	if (!help || help[0] == '\0')
		return false;
	if (argument != ArgumentNone && type == OptionNone)
		return false;

	/* Reject duplicate options. */
	if (optionsMap_.find(name) != optionsMap_.end())
		return false;

	optionsMap_[name] = Option({ 0, type, name, argument, nullptr,
				     help, nullptr });
	return true;
}

KeyValueParser::Options KeyValueParser::parse(const char *arguments)
{
	Options options;

	for (const char *pair = arguments; *arguments != '\0'; pair = arguments) {
		const char *comma = strchrnul(arguments, ',');
		size_t len = comma - pair;

		/* Skip over the comma. */
		arguments = *comma == ',' ? comma + 1 : comma;

		/* Skip to the next pair if the pair is empty. */
		if (!len)
			continue;

		std::string key;
		std::string value;

		const char *separator = static_cast<const char *>(memchr(pair, '=', len));
		if (!separator) {
			key = std::string(pair, len);
			value = "";
		} else {
			key = std::string(pair, separator - pair);
			value = std::string(separator + 1, comma - separator - 1);
		}

		/* The key is mandatory, the value might be optional. */
		if (key.empty())
			continue;

		if (optionsMap_.find(key) == optionsMap_.end()) {
			std::cerr << "Invalid option " << key << std::endl;
			options.clear();
			break;
		}

		OptionArgument arg = optionsMap_[key].argument;
		if (value.empty() && arg == ArgumentRequired) {
			std::cerr << "Option " << key << " requires an argument"
				  << std::endl;
			options.clear();
			break;
		} else if (!value.empty() && arg == ArgumentNone) {
			std::cerr << "Option " << key << " takes no argument"
				  << std::endl;
			options.clear();
			break;
		}

		const Option &option = optionsMap_[key];
		if (!options.parseValue(key, option, value.c_str())) {
			std::cerr << "Failed to parse '" << value << "' as "
				  << option.typeName() << " for option " << key
				  << std::endl;
			options.clear();
			break;
		}
	}

	return options;
}

void KeyValueParser::usage(int indent)
{
	unsigned int space = 0;

	for (auto const &iter : optionsMap_) {
		const Option &option = iter.second;
		unsigned int length = 14;
		if (option.argument != ArgumentNone)
			length += 1 + strlen(option.typeName());
		if (option.argument == ArgumentOptional)
			length += 2;

		if (length > space)
			space = length;
	}

	space = (space + 7) / 8 * 8;

	for (auto const &iter : optionsMap_) {
		const Option &option = iter.second;
		std::string argument = option.name;

		if (option.argument != ArgumentNone) {
			if (option.argument == ArgumentOptional)
				argument += "[=";
			else
				argument += "=";
			argument += option.typeName();
			if (option.argument == ArgumentOptional)
				argument += "]";
		}

		std::cerr << std::setw(indent) << std::right << " "
			  << std::setw(space) << std::left << argument;

		for (const char *help = option.help, *end = help; end;) {
			end = strchr(help, '\n');
			if (end) {
				std::cerr << std::string(help, end - help + 1);
				std::cerr << std::setw(indent + space) << " ";
				help = end + 1;
			} else {
				std::cerr << help << std::endl;
			}
		}
	}
}

/* -----------------------------------------------------------------------------
 * OptionValue
 */

OptionValue::OptionValue()
	: type_(OptionNone)
{
}

OptionValue::OptionValue(int value)
	: type_(OptionInteger), integer_(value)
{
}

OptionValue::OptionValue(const char *value)
	: type_(OptionString), string_(value)
{
}

OptionValue::OptionValue(const std::string &value)
	: type_(OptionString), string_(value)
{
}

OptionValue::OptionValue(const KeyValueParser::Options &value)
	: type_(OptionKeyValue), keyValues_(value)
{
}

OptionValue::operator int() const
{
	if (type_ != OptionInteger)
		return 0;

	return integer_;
}

OptionValue::operator std::string() const
{
	if (type_ != OptionString)
		return std::string();

	return string_;
}

OptionValue::operator KeyValueParser::Options() const
{
	if (type_ != OptionKeyValue)
		return KeyValueParser::Options();

	return keyValues_;
}

/* -----------------------------------------------------------------------------
 * OptionsParser
 */

bool OptionsParser::addOption(int opt, OptionType type, const char *help,
			      const char *name, OptionArgument argument,
			      const char *argumentName)
{
	/*
	 * Options must have at least a short or long name, and a text message.
	 * If an argument is accepted, it must be described by argumentName.
	 */
	if (!isalnum(opt) && !name)
		return false;
	if (!help || help[0] == '\0')
		return false;
	if (argument != ArgumentNone && !argumentName)
		return false;

	/* Reject duplicate options. */
	if (optionsMap_.find(opt) != optionsMap_.end())
		return false;

	options_.push_back(Option({ opt, type, name, argument, argumentName,
				    help, nullptr }));
	optionsMap_[opt] = &options_.back();
	return true;
}

bool OptionsParser::addOption(int opt, KeyValueParser *parser, const char *help,
			      const char *name)
{
	if (!addOption(opt, OptionKeyValue, help, name, ArgumentRequired,
		       "key=value[,key=value,...]"))
		return false;

	options_.back().keyValueParser = parser;
	return true;
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

		const Option &option = *optionsMap_[c];
		if (!options.parseValue(c, option, optarg)) {
			parseValueError(option);
			usage();
			options.clear();
			break;
		}
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
			if (option.argument == ArgumentOptional)
				argument += "[=";
			else
				argument += " ";
			argument += option.argumentName;
			if (option.argument == ArgumentOptional)
				argument += "]";
		}

		std::cerr << std::setw(indent) << std::left << argument;

		for (const char *help = option.help, *end = help; end; ) {
			end = strchr(help, '\n');
			if (end) {
				std::cerr << std::string(help, end - help + 1);
				std::cerr << std::setw(indent) << " ";
				help = end + 1;
			} else {
				std::cerr << help << std::endl;
			}
		}

		if (option.keyValueParser)
			option.keyValueParser->usage(indent);
	}
}

void OptionsParser::parseValueError(const Option &option)
{
	std::string optionName;

	if (option.name)
		optionName = "--" + std::string(option.name);
	else
		optionName = "-" + static_cast<char>(option.opt);

	std::cerr << "Can't parse " << option.typeName()
		  << " argument for option " << optionName << std::endl;
}
