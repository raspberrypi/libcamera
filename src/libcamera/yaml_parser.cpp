/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * libcamera YAML parsing helper
 */

#include "libcamera/internal/yaml_parser.h"

#include <errno.h>
#include <functional>
#include <memory>
#include <string>

#include <libcamera/base/file.h>
#include <libcamera/base/log.h>

#include <yaml.h>

/**
 * \file yaml_parser.h
 * \brief A YAML parser helper
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(YamlParser)

#ifndef __DOXYGEN__

class YamlParserContext
{
public:
	YamlParserContext();
	~YamlParserContext();

	int init(File &file);
	int parseContent(ValueNode &valueNode);

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

	std::string readValue(const EventPtr &event);
	int parseDictionaryOrList(yaml_event_type_t endEventType,
				  const std::function<int(EventPtr event)> &parseItem);
	int parseNextNode(ValueNode &valueNode, EventPtr event);

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
	if (!yaml_parser_parse(&parser_, event.get())) {
		File *file = static_cast<File *>(parser_.read_handler_data);

		LOG(YamlParser, Error) << file->fileName() << ":"
				       << parser_.problem_mark.line << ":"
				       << parser_.problem_mark.column << " "
				       << parser_.problem << " "
				       << parser_.context;

		return nullptr;
	}

	return event;
}

/**
 * \brief Parse the content of a YAML document
 * \param[in] valueNode The result of ValueNode
 *
 * Check YAML start and end events of a YAML document, and parse the root object
 * of the YAML document into a ValueNode.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL The parser has failed to validate end of a YAML file
 */
int YamlParserContext::parseContent(ValueNode &valueNode)
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
	if (parseNextNode(valueNode, std::move(event)))
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
 * \brief Parse event scalar and fill its content into a string
 *
 * A helper function to parse a scalar event as string. The caller needs to
 * guarantee the event is of scalar type.
 *
 * \return The scalar event value as a string
 */
std::string YamlParserContext::readValue(const EventPtr &event)
{
	return std::string{ reinterpret_cast<const char *>(event->data.scalar.value),
			    event->data.scalar.length };
}

/**
 * \brief A helper function to abstract the common part of parsing dictionary or list
 * \param[in] endEventType The YAML end event type (sequence or mapping)
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
int YamlParserContext::parseDictionaryOrList(yaml_event_type_t endEventType,
					     const std::function<int(EventPtr event)> &parseItem)
{
	/*
	 * Add a safety counter to make sure we don't loop indefinitely in case
	 * the YAML file is malformed.
	 */
	for (unsigned int sentinel = 2000; sentinel; sentinel--) {
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
 * \brief Parse next YAML event and read it as a ValueNode
 * \param[in] valueNode The result of ValueNode
 * \param[in] event The leading event of the object
 *
 * Parse next YAML object separately as a value, list or dictionary.
 *
 * \return 0 on success or a negative error code otherwise
 * \retval -EINVAL Fail to parse the YAML file.
 */
int YamlParserContext::parseNextNode(ValueNode &valueNode, EventPtr event)
{
	if (!event)
		return -EINVAL;

	switch (event->type) {
	case YAML_SCALAR_EVENT:
		valueNode.set(readValue(event));
		return 0;

	case YAML_SEQUENCE_START_EVENT: {
		auto handler = [this, &valueNode](EventPtr evt) {
			ValueNode *child = valueNode.add(std::make_unique<ValueNode>());
			return parseNextNode(*child, std::move(evt));
		};
		return parseDictionaryOrList(YAML_SEQUENCE_END_EVENT, handler);
	}

	case YAML_MAPPING_START_EVENT: {
		auto handler = [this, &valueNode](EventPtr evtKey) {
			/* Parse key */
			if (evtKey->type != YAML_SCALAR_EVENT) {
				LOG(YamlParser, Error) << "Expect key at line: "
						       << evtKey->start_mark.line
						       << " column: "
						       << evtKey->start_mark.column;
				return -EINVAL;
			}

			std::string key = readValue(evtKey);

			/* Parse value */
			EventPtr evtValue = nextEvent();
			if (!evtValue)
				return -EINVAL;

			ValueNode *child = valueNode.add(std::move(key),
							 std::make_unique<ValueNode>());
			if (!child) {
				LOG(YamlParser, Error)
					<< "Duplicated key at line "
					<< evtKey->start_mark.line;
				return -EINVAL;
			}

			return parseNextNode(*child, std::move(evtValue));
		};
		int ret = parseDictionaryOrList(YAML_MAPPING_END_EVENT, handler);
		if (ret)
			return ret;

		return 0;
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
 * YAML file into a tree of ValueNode instances.
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
 * std::unique_ptr<ValueNode> root = YamlParser::parse(fh);
 * if (!root)
 *   return;
 *
 * if (!root->isDictionary())
 *   return;
 *
 * const ValueNode &name = (*root)["name"];
 * std::cout << name.get<std::string>("") << std::endl;
 *
 * const ValueNode &numbers = (*root)["numbers"];
 * if (!numbers.isList())
 *   return;
 *
 * for (std::size_t i = 0; i < numbers.size(); i++)
 *   std::cout << numbers[i].get<int32_t>(0) << std::endl;
 *
 * \endcode
 *
 * The YamlParser::parse() function takes an open FILE, parses its contents, and
 * returns a pointer to a ValueNode corresponding to the root node of the YAML
 * document.
 *
 * The parser preserves the order of items in the YAML file, for both lists and
 * dictionaries.
 */

/**
 * \brief Parse a YAML file as a ValueNode
 * \param[in] file The YAML file to parse
 *
 * The YamlParser::parse() function takes a file, parses its contents, and
 * returns a pointer to a ValueNode corresponding to the root node of the YAML
 * document.
 *
 * \return Pointer to result ValueNode on success or nullptr otherwise
 */
std::unique_ptr<ValueNode> YamlParser::parse(File &file)
{
	YamlParserContext context;

	if (context.init(file))
		return nullptr;

	std::unique_ptr<ValueNode> root = std::make_unique<ValueNode>();

	if (context.parseContent(*root)) {
		LOG(YamlParser, Error)
			<< "Failed to parse YAML content from "
			<< file.fileName();
		return nullptr;
	}

	return root;
}

} /* namespace libcamera */
