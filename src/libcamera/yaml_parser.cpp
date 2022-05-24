/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * yaml_parser.cpp - libcamera YAML parsing helper
 */

#include "libcamera/internal/yaml_parser.h"

#include <cstdlib>
#include <errno.h>
#include <functional>
#include <limits>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <yaml.h>

/**
 * \file libcamera/internal/yaml_parser.h
 * \brief A YAML parser helper
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(YamlParser)

namespace {

/* Empty static YamlObject as a safe result for invalid operations */
static const YamlObject empty;

void setOk(bool *ok, bool result)
{
	if (ok)
		*ok = result;
}

} /* namespace */

/**
 * \class YamlObject
 * \brief A class representing the tree structure of the YAML content
 *
 * The YamlObject class represents the tree structure of YAML content. A
 * YamlObject can be a dictionary or list of YamlObjects or a value if a tree
 * leaf.
 */

YamlObject::YamlObject()
	: type_(Type::Value)
{
}

YamlObject::~YamlObject() = default;

/**
 * \fn YamlObject::isValue()
 * \brief Return whether the YamlObject is a value
 *
 * \return True if the YamlObject is a value, false otherwise
 */

/**
 * \fn YamlObject::isList()
 * \brief Return whether the YamlObject is a list
 *
 * \return True if the YamlObject is a list, false otherwise
 */

/**
 * \fn YamlObject::isDictionary()
 * \brief Return whether the YamlObject is a dictionary
 *
 * \return True if the YamlObject is a dictionary, false otherwise
 */

/**
 * \fn YamlObject::size()
 * \brief Retrieve the number of elements in a dictionary or list YamlObject
 *
 * This function retrieves the size of the YamlObject, defined as the number of
 * child elements it contains. Only YamlObject instances of Dictionary or List
 * types have a size, calling this function on other types of instances is
 * invalid and results in undefined behaviour.
 *
 * \return The size of the YamlObject
 */
std::size_t YamlObject::size() const
{
	switch (type_) {
	case Type::Dictionary:
		return dictionary_.size();
	case Type::List:
		return list_.size();
	default:
		return 0;
	}
}

/**
 * \fn template<typename T> YamlObject::get<T>(
 *	const T &defaultValue, bool *ok) const
 * \brief Parse the YamlObject as a \a T value
 * \param[in] defaultValue The default value when failing to parse
 * \param[out] ok The result of whether the parse succeeded
 *
 * This function parses the value of the YamlObject as a \a T object, and
 * returns the value. If parsing fails (usually because the YamlObject doesn't
 * store a \a T value), the \a defaultValue is returned, and \a ok is set to
 * false. Otherwise, the YamlObject value is returned, and \a ok is set to true.
 *
 * The \a ok pointer is optional and can be a nullptr if the caller doesn't
 * need to know if parsing succeeded.
 *
 * \return Value as a bool type
 */

#ifndef __DOXYGEN__

template<>
bool YamlObject::get(const bool &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	if (value_ == "true") {
		setOk(ok, true);
		return true;
	} else if (value_ == "false") {
		setOk(ok, true);
		return false;
	}

	return defaultValue;
}

template<>
int16_t YamlObject::get(const int16_t &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	if (value_ == "")
		return defaultValue;

	char *end;

	errno = 0;
	int16_t value = std::strtol(value_.c_str(), &end, 10);

	if ('\0' != *end || errno == ERANGE ||
	    value < std::numeric_limits<int16_t>::min() ||
	    value > std::numeric_limits<int16_t>::max())
		return defaultValue;

	setOk(ok, true);
	return value;
}

template<>
uint16_t YamlObject::get(const uint16_t &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	if (value_ == "")
		return defaultValue;

	/*
	 * libyaml parses all scalar values as strings. When a string has
	 * leading spaces before a minus sign, for example "  -10", strtoul
	 * skips leading spaces, accepts the leading minus sign, and the
	 * calculated digits are negated as if by unary minus. Rule it out in
	 * case the user gets a large number when the value is negative.
	 */
	std::size_t found = value_.find_first_not_of(" \t");
	if (found != std::string::npos && value_[found] == '-')
		return defaultValue;

	char *end;

	errno = 0;
	uint16_t value = std::strtoul(value_.c_str(), &end, 10);

	if ('\0' != *end || errno == ERANGE ||
	    value < std::numeric_limits<uint16_t>::min() ||
	    value > std::numeric_limits<uint16_t>::max())
		return defaultValue;

	setOk(ok, true);
	return value;
}

template<>
int32_t YamlObject::get(const int32_t &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	if (value_ == "")
		return defaultValue;

	char *end;

	errno = 0;
	long value = std::strtol(value_.c_str(), &end, 10);

	if ('\0' != *end || errno == ERANGE ||
	    value < std::numeric_limits<int32_t>::min() ||
	    value > std::numeric_limits<int32_t>::max())
		return defaultValue;

	setOk(ok, true);
	return value;
}

template<>
uint32_t YamlObject::get(const uint32_t &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	if (value_ == "")
		return defaultValue;

	/*
	 * libyaml parses all scalar values as strings. When a string has
	 * leading spaces before a minus sign, for example "  -10", strtoul
	 * skips leading spaces, accepts the leading minus sign, and the
	 * calculated digits are negated as if by unary minus. Rule it out in
	 * case the user gets a large number when the value is negative.
	 */
	std::size_t found = value_.find_first_not_of(" \t");
	if (found != std::string::npos && value_[found] == '-')
		return defaultValue;

	char *end;

	errno = 0;
	unsigned long value = std::strtoul(value_.c_str(), &end, 10);

	if ('\0' != *end || errno == ERANGE ||
	    value < std::numeric_limits<uint32_t>::min() ||
	    value > std::numeric_limits<uint32_t>::max())
		return defaultValue;

	setOk(ok, true);
	return value;
}

template<>
double YamlObject::get(const double &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	if (value_ == "")
		return defaultValue;

	char *end;

	errno = 0;
	double value = std::strtod(value_.c_str(), &end);

	if ('\0' != *end || errno == ERANGE)
		return defaultValue;

	setOk(ok, true);
	return value;
}

template<>
std::string YamlObject::get(const std::string &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::Value)
		return defaultValue;

	setOk(ok, true);
	return value_;
}

template<>
Size YamlObject::get(const Size &defaultValue, bool *ok) const
{
	setOk(ok, false);

	if (type_ != Type::List)
		return defaultValue;

	if (list_.size() != 2)
		return defaultValue;

	/*
	 * Add a local variable to validate each dimension in case
	 * that ok == nullptr.
	 */
	bool valid;
	uint32_t width = list_[0]->get<uint32_t>(0, &valid);
	if (!valid)
		return defaultValue;

	uint32_t height = list_[1]->get<uint32_t>(0, &valid);
	if (!valid)
		return defaultValue;

	setOk(ok, true);
	return Size(width, height);
}

#endif /* __DOXYGEN__ */

/**
 * \fn YamlObject::asDict() const
 * \brief Wrap a dictionary YamlObject in an adapter that exposes iterators
 *
 * The YamlObject class doesn't directly implement iterators, as the iterator
 * type depends on whether the object is a Dictionary or List. This function
 * wraps a YamlObject of Dictionary type into an adapter that exposes
 * iterators, as well as begin() and end() functions, allowing usage of
 * range-based for loops with YamlObject. As YAML mappings are not ordered, the
 * iteration order is not specified.
 *
 * The iterator's value_type is a
 * <em>std::pair<const std::string &, const \ref YamlObject &></em>.
 *
 * If the YamlObject is not of Dictionary type, the returned adapter operates
 * as an empty container.
 *
 * \return An adapter of unspecified type compatible with range-based for loops
 */

/**
 * \fn YamlObject::asList() const
 * \brief Wrap a list YamlObject in an adapter that exposes iterators
 *
 * The YamlObject class doesn't directly implement iterators, as the iterator
 * type depends on whether the object is a Dictionary or List. This function
 * wraps a YamlObject of List type into an adapter that exposes iterators, as
 * well as begin() and end() functions, allowing usage of range-based for loops
 * with YamlObject. As YAML lists are ordered, the iteration order is identical
 * to the list order in the YAML data.
 *
 * The iterator's value_type is a <em>const YamlObject &</em>.
 *
 * If the YamlObject is not of List type, the returned adapter operates as an
 * empty container.
 *
 * \return An adapter of unspecified type compatible with range-based for loops
 */

/**
 * \fn YamlObject::operator[](std::size_t index) const
 * \brief Retrieve the element from list YamlObject by index
 *
 * This function retrieves an element of the YamlObject. Only YamlObject
 * instances of List type associate elements with index, calling this function
 * on other types of instances is invalid and results in undefined behaviour.
 *
 * \return The YamlObject as an element of the list
 */
const YamlObject &YamlObject::operator[](std::size_t index) const
{
	if (type_ != Type::List || index >= size())
		return empty;

	return *list_[index];
}

/**
 * \fn YamlObject::contains()
 * \brief Check if an element of a dictionary exists
 *
 * This function check if the YamlObject contains an element. Only YamlObject
 * instances of Dictionary type associate elements with names, calling this
 * function on other types of instances is invalid and results in undefined
 * behaviour.
 *
 * \return True if an element exists, false otherwise
 */
bool YamlObject::contains(const std::string &key) const
{
	if (dictionary_.find(key) == dictionary_.end())
		return false;

	return true;
}

/**
 * \fn YamlObject::operator[](const std::string &key) const
 * \brief Retrieve a member by name from the dictionary
 *
 * This function retrieve a member of a YamlObject by name. Only YamlObject
 * instances of Dictionary type associate elements with names, calling this
 * function on other types of instances is invalid and results in undefined
 * behaviour.
 *
 * \return The YamlObject corresponding to the \a key member
 */
const YamlObject &YamlObject::operator[](const std::string &key) const
{
	if (type_ != Type::Dictionary || !contains(key))
		return empty;

	auto iter = dictionary_.find(key);
	return *iter->second;
}

#ifndef __DOXYGEN__

class YamlParserContext
{
public:
	YamlParserContext();
	~YamlParserContext();

	int init(File &file);
	int parseContent(YamlObject &yamlObject);

private:
	struct EventDeleter {
		void operator()(yaml_event_t *event) const
		{
			yaml_event_delete(event);
			delete event;
		}
	};
	using EventPtr = std::unique_ptr<yaml_event_t, EventDeleter>;

	static int yamlRead(void *data, unsigned char *buffer, size_t size,
			    size_t *sizeRead);

	EventPtr nextEvent();

	void readValue(std::string &value, EventPtr event);
	int parseDictionaryOrList(YamlObject::Type type,
				  const std::function<int(EventPtr event)> &parseItem);
	int parseNextYamlObject(YamlObject &yamlObject, EventPtr event);

	bool parserValid_;
	yaml_parser_t parser_;
};

/**
 * \class YamlParserContext
 * \brief Class for YamlParser parsing and context data
 *
 * The YamlParserContext class stores the internal yaml_parser_t and provides
 * helper functions to do event-based parsing for YAML files.
 */
YamlParserContext::YamlParserContext()
	: parserValid_(false)
{
}

/**
 * \class YamlParserContext
 * \brief Destructor of YamlParserContext
 */
YamlParserContext::~YamlParserContext()
{
	if (parserValid_) {
		yaml_parser_delete(&parser_);
		parserValid_ = false;
	}
}

/**
 * \fn YamlParserContext::init()
 * \brief Initialize a parser with an opened file for parsing
 * \param[in] fh The YAML file to parse
 *
 * Prior to parsing the YAML content, the YamlParserContext must be initialized
 * with a file to create an internal parser. The file needs to stay valid until
 * parsing completes.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL The parser has failed to initialize
 */
int YamlParserContext::init(File &file)
{
	/* yaml_parser_initialize returns 1 when it succeededs */
	if (!yaml_parser_initialize(&parser_)) {
		LOG(YamlParser, Error) << "Failed to initialize YAML parser";
		return -EINVAL;
	}
	parserValid_ = true;
	yaml_parser_set_input(&parser_, &YamlParserContext::yamlRead, &file);

	return 0;
}

int YamlParserContext::yamlRead(void *data, unsigned char *buffer, size_t size,
				size_t *sizeRead)
{
	File *file = static_cast<File *>(data);

	Span<unsigned char> buf{ buffer, size };
	ssize_t ret = file->read(buf);
	if (ret < 0)
		return 0;

	*sizeRead = ret;
	return 1;
}

/**
 * \fn YamlParserContext::nextEvent()
 * \brief Get the next event
 *
 * Get the next event in the current YAML event stream, and return nullptr when
 * there is no more event.
 *
 * \return The next event on success or nullptr otherwise
 */
YamlParserContext::EventPtr YamlParserContext::nextEvent()
{
	EventPtr event(new yaml_event_t);

	/* yaml_parser_parse returns 1 when it succeeds */
	if (!yaml_parser_parse(&parser_, event.get()))
		return nullptr;

	return event;
}

/**
 * \fn YamlParserContext::parseContent()
 * \brief Parse the content of a YAML document
 * \param[in] yamlObject The result of YamlObject
 *
 * Check YAML start and end events of a YAML document, and parse the root object
 * of the YAML document into a YamlObject.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL The parser has failed to validate end of a YAML file
 */
int YamlParserContext::parseContent(YamlObject &yamlObject)
{
	/* Check start of the YAML file. */
	EventPtr event = nextEvent();
	if (!event || event->type != YAML_STREAM_START_EVENT)
		return -EINVAL;

	event = nextEvent();
	if (!event || event->type != YAML_DOCUMENT_START_EVENT)
		return -EINVAL;

	/* Parse the root object. */
	event = nextEvent();
	if (parseNextYamlObject(yamlObject, std::move(event)))
		return -EINVAL;

	/* Check end of the YAML file. */
	event = nextEvent();
	if (!event || event->type != YAML_DOCUMENT_END_EVENT)
		return -EINVAL;

	event = nextEvent();
	if (!event || event->type != YAML_STREAM_END_EVENT)
		return -EINVAL;

	return 0;
}

/**
 * \fn YamlParserContext::readValue()
 * \brief Parse event scalar and fill its content into a string
 * \param[in] value The string reference to fill value
 *
 * A helper function to parse a scalar event as string. The caller needs to
 * guarantee the event is of scaler type.
 */
void YamlParserContext::readValue(std::string &value, EventPtr event)
{
	value.assign(reinterpret_cast<char *>(event->data.scalar.value),
		     event->data.scalar.length);
}

/**
 * \fn YamlParserContext::parseDictionaryOrList()
 * \brief A helper function to abstract the common part of parsing dictionary or list
 *
 * \param[in] isDictionary True for parsing a dictionary, and false for a list
 * \param[in] parseItem The callback to handle an item
 *
 * A helper function to abstract parsing an item from a dictionary or a list.
 * The differences of them in a YAML event stream are:
 *
 * 1. The start and end event types are different
 * 2. There is a leading scalar string as key in the items of a dictionary
 *
 * The caller should handle the leading key string in its callback parseItem
 * when it's a dictionary.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL The parser is failed to initialize
 */
int YamlParserContext::parseDictionaryOrList(YamlObject::Type type,
					     const std::function<int(EventPtr event)> &parseItem)
{
	yaml_event_type_t endEventType = YAML_SEQUENCE_END_EVENT;
	if (type == YamlObject::Type::Dictionary)
		endEventType = YAML_MAPPING_END_EVENT;

	/*
	 * Add a safety counter to make sure we don't loop indefinitely in case
	 * the YAML file is malformed.
	 */
	for (unsigned int sentinel = 1000; sentinel; sentinel--) {
		auto evt = nextEvent();
		if (!evt)
			return -EINVAL;

		if (evt->type == endEventType)
			return 0;

		int ret = parseItem(std::move(evt));
		if (ret)
			return ret;
	}

	LOG(YamlParser, Error) << "The YAML file contains a List or Dictionary"
				  " whose size exceeds the parser's limit (1000)";

	return -EINVAL;
}

/**
 * \fn YamlParserContext::parseNextYamlObject()
 * \brief Parse next YAML event and read it as a YamlObject
 * \param[in] yamlObject The result of YamlObject
 * \param[in] event The leading event of the object
 *
 * Parse next YAML object separately as a value, list or dictionary.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL Fail to parse the YAML file.
 */
int YamlParserContext::parseNextYamlObject(YamlObject &yamlObject, EventPtr event)
{
	if (!event)
		return -EINVAL;

	switch (event->type) {
	case YAML_SCALAR_EVENT:
		yamlObject.type_ = YamlObject::Type::Value;
		readValue(yamlObject.value_, std::move(event));
		return 0;

	case YAML_SEQUENCE_START_EVENT: {
		yamlObject.type_ = YamlObject::Type::List;
		auto &list = yamlObject.list_;
		auto handler = [this, &list](EventPtr evt) {
			list.emplace_back(new YamlObject());
			return parseNextYamlObject(*list.back(), std::move(evt));
		};
		return parseDictionaryOrList(YamlObject::Type::List, handler);
	}

	case YAML_MAPPING_START_EVENT: {
		yamlObject.type_ = YamlObject::Type::Dictionary;
		auto &dictionary = yamlObject.dictionary_;
		auto handler = [this, &dictionary](EventPtr evtKey) {
			/* Parse key */
			if (evtKey->type != YAML_SCALAR_EVENT) {
				LOG(YamlParser, Error) << "Expect key at line: "
						       << evtKey->start_mark.line
						       << " column: "
						       << evtKey->start_mark.column;
				return -EINVAL;
			}

			std::string key;
			readValue(key, std::move(evtKey));

			/* Parse value */
			EventPtr evtValue = nextEvent();
			if (!evtValue)
				return -EINVAL;

			auto elem = dictionary.emplace(key, std::make_unique<YamlObject>());
			return parseNextYamlObject(*elem.first->second.get(), std::move(evtValue));
		};
		return parseDictionaryOrList(YamlObject::Type::Dictionary, handler);
	}

	default:
		LOG(YamlParser, Error) << "Invalid YAML file";
		return -EINVAL;
	}
}

#endif /* __DOXYGEN__ */

/**
 * \class YamlParser
 * \brief A helper class for parsing a YAML file
 *
 * The YamlParser class provides an easy interface to parse the contents of a
 * YAML file into a tree of YamlObject instances.
 *
 * Example usage:
 *
 * \code{.unparsed}
 *
 * name:
 * 	"John"
 * numbers:
 * 	- 1
 * 	- 2
 *
 * \endcode
 *
 * The following code illustrates how to parse the above YAML file:
 *
 * \code{.cpp}
 *
 * std::unique_ptr<YamlObject> root = YamlParser::parse(fh);
 * if (!root)
 *   return;
 *
 * if (!root->isDictionary())
 *   return;
 *
 * const YamlObject &name = (*root)["name"];
 * std::cout << name.get<std::string>("") << std::endl;
 *
 * const YamlObject &numbers = (*root)["numbers"];
 * if (!numbers.isList())
 *   return;
 *
 * for (std::size_t i = 0; i < numbers.size(); i++)
 *   std::cout << numbers[i].get<int32_t>(0) << std::endl;
 *
 * \endcode
 *
 * The YamlParser::parse() function takes an open FILE, parses its contents, and
 * returns a pointer to a YamlObject corresponding to the root node of the YAML
 * document.
 */

/**
 * \brief Parse a YAML file as a YamlObject
 * \param[in] file The YAML file to parse
 *
 * The YamlParser::parse() function takes a file, parses its contents, and
 * returns a pointer to a YamlObject corresponding to the root node of the YAML
 * document.
 *
 * \return Pointer to result YamlObject on success or nullptr otherwise
 */
std::unique_ptr<YamlObject> YamlParser::parse(File &file)
{
	YamlParserContext context;

	if (context.init(file))
		return nullptr;

	std::unique_ptr<YamlObject> root(new YamlObject());

	if (context.parseContent(*root)) {
		LOG(YamlParser, Error) << "Failed to parse YAML content";
		return nullptr;
	}

	return root;
}

} /* namespace libcamera */
