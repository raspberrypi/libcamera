/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <map>
#include <signal.h>
#include <sstream>
#include <string.h>

#include <libcamera/libcamera.h>

#include "buffer_writer.h"
#include "event_loop.h"
#include "options.h"

using namespace libcamera;

OptionsParser::Options options;
std::shared_ptr<Camera> camera;
std::map<Stream *, std::string> streamInfo;
EventLoop *loop;
BufferWriter *writer;

enum {
	OptCamera = 'c',
	OptCapture = 'C',
	OptFile = 'F',
	OptHelp = 'h',
	OptList = 'l',
	OptStream = 's',
};

void signalHandler(int signal)
{
	std::cout << "Exiting" << std::endl;
	loop->exit();
}

static int parseOptions(int argc, char *argv[])
{
	KeyValueParser streamKeyValue;
	streamKeyValue.addOption("role", OptionString,
				 "Role for the stream (viewfinder, video, still)",
				 ArgumentRequired);
	streamKeyValue.addOption("width", OptionInteger, "Width in pixels",
				 ArgumentRequired);
	streamKeyValue.addOption("height", OptionInteger, "Height in pixels",
				 ArgumentRequired);
	streamKeyValue.addOption("pixelformat", OptionInteger, "Pixel format",
				 ArgumentRequired);

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptCapture, OptionNone,
			 "Capture until interrupted by user", "capture");
	parser.addOption(OptFile, OptionString,
			 "Write captured frames to disk\n"
			 "The first '#' character in the file name is expanded to the stream name and frame sequence number.\n"
			 "The default file name is 'frame-#.bin'.",
			 "file", ArgumentOptional, "filename");
	parser.addOption(OptStream, &streamKeyValue,
			 "Set configuration of a camera stream", "stream", true);
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");
	parser.addOption(OptList, OptionNone, "List all cameras", "list");

	options = parser.parse(argc, argv);
	if (!options.valid())
		return -EINVAL;

	if (options.empty() || options.isSet(OptHelp)) {
		parser.usage();
		return options.empty() ? -EINVAL : -EINTR;
	}

	return 0;
}

static int prepareCameraConfig(CameraConfiguration *config)
{
	StreamRoles roles;

	streamInfo.clear();

	/* If no configuration is provided assume a single video stream. */
	if (!options.isSet(OptStream)) {
		*config = camera->generateConfiguration({ StreamRole::VideoRecording });
		streamInfo[config->front()] = "stream0";
		return 0;
	}

	const std::vector<OptionValue> &streamOptions =
		options[OptStream].toArray();

	/* Use roles and get a default configuration. */
	for (auto const &value : streamOptions) {
		KeyValueParser::Options conf = value.toKeyValues();

		if (!conf.isSet("role")) {
			roles.push_back(StreamRole::VideoRecording);
		} else if (conf["role"].toString() == "viewfinder") {
			roles.push_back(StreamRole::Viewfinder);
		} else if (conf["role"].toString() == "video") {
			roles.push_back(StreamRole::VideoRecording);
		} else if (conf["role"].toString() == "still") {
			roles.push_back(StreamRole::StillCapture);
		} else {
			std::cerr << "Unknown stream role "
				  << conf["role"].toString() << std::endl;
			return -EINVAL;
		}
	}

	*config = camera->generateConfiguration(roles);

	if (!config->isValid()) {
		std::cerr << "Failed to get default stream configuration"
			  << std::endl;
		return -EINVAL;
	}

	/* Apply configuration explicitly requested. */
	CameraConfiguration::iterator it = config->begin();
	for (auto const &value : streamOptions) {
		KeyValueParser::Options conf = value.toKeyValues();
		Stream *stream = *it;
		it++;

		if (conf.isSet("width"))
			(*config)[stream].size.width = conf["width"];

		if (conf.isSet("height"))
			(*config)[stream].size.height = conf["height"];

		/* TODO: Translate 4CC string to ID. */
		if (conf.isSet("pixelformat"))
			(*config)[stream].pixelFormat = conf["pixelformat"];
	}

	unsigned int index = 0;
	for (Stream *stream : *config) {
		streamInfo[stream] = "stream" + std::to_string(index);
		index++;
	}

	return 0;
}

static void requestComplete(Request *request, const std::map<Stream *, Buffer *> &buffers)
{
	static uint64_t now, last = 0;
	double fps = 0.0;

	if (request->status() == Request::RequestCancelled)
		return;

	struct timespec time;
	clock_gettime(CLOCK_MONOTONIC, &time);
	now = time.tv_sec * 1000 + time.tv_nsec / 1000000;
	fps = now - last;
	fps = last && fps ? 1000.0 / fps : 0.0;
	last = now;

	std::stringstream info;
	info << "fps: " << std::fixed << std::setprecision(2) << fps;

	for (auto it = buffers.begin(); it != buffers.end(); ++it) {
		Stream *stream = it->first;
		Buffer *buffer = it->second;
		const std::string &name = streamInfo[stream];

		info << " " << name
		     << " (" << buffer->index() << ")"
		     << " seq: " << std::setw(6) << std::setfill('0') << buffer->sequence()
		     << " bytesused: " << buffer->bytesused();

		if (writer)
			writer->write(buffer, name);
	}

	std::cout << info.str() << std::endl;

	request = camera->createRequest();
	if (!request) {
		std::cerr << "Can't create request" << std::endl;
		return;
	}

	request->setBuffers(buffers);
	camera->queueRequest(request);
}

static int capture()
{
	CameraConfiguration config;
	int ret;

	ret = prepareCameraConfig(&config);
	if (ret) {
		std::cout << "Failed to prepare camera configuration" << std::endl;
		return ret;
	}

	ret = camera->configure(config);
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	ret = camera->allocateBuffers();
	if (ret) {
		std::cerr << "Failed to allocate buffers"
			  << std::endl;
		return ret;
	}

	camera->requestCompleted.connect(requestComplete);

	/* Identify the stream with the least number of buffers. */
	unsigned int nbuffers = UINT_MAX;
	for (Stream *stream : config)
		nbuffers = std::min(nbuffers, stream->bufferPool().count());

	/*
	 * TODO: make cam tool smarter to support still capture by for
	 * example pushing a button. For now run all streams all the time.
	 */

	std::vector<Request *> requests;
	for (unsigned int i = 0; i < nbuffers; i++) {
		Request *request = camera->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			ret = -ENOMEM;
			goto out;
		}

		std::map<Stream *, Buffer *> map;
		for (Stream *stream : config)
			map[stream] = &stream->bufferPool().buffers()[i];

		ret = request->setBuffers(map);
		if (ret < 0) {
			std::cerr << "Can't set buffers for request" << std::endl;
			goto out;
		}

		requests.push_back(request);
	}

	ret = camera->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		goto out;
	}

	for (Request *request : requests) {
		ret = camera->queueRequest(request);
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
			goto out;
		}
	}

	std::cout << "Capture until user interrupts by SIGINT" << std::endl;
	ret = loop->exec();

	ret = camera->stop();
	if (ret)
		std::cout << "Failed to stop capture" << std::endl;
out:
	camera->freeBuffers();

	return ret;
}

int main(int argc, char **argv)
{
	int ret;

	ret = parseOptions(argc, argv);
	if (ret < 0)
		return ret == -EINTR ? 0 : EXIT_FAILURE;

	CameraManager *cm = CameraManager::instance();

	ret = cm->start();
	if (ret) {
		std::cout << "Failed to start camera manager: "
			  << strerror(-ret) << std::endl;
		return EXIT_FAILURE;
	}

	loop = new EventLoop(cm->eventDispatcher());

	struct sigaction sa = {};
	sa.sa_handler = &signalHandler;
	sigaction(SIGINT, &sa, nullptr);

	if (options.isSet(OptList)) {
		std::cout << "Available cameras:" << std::endl;
		for (const std::shared_ptr<Camera> &cam : cm->cameras())
			std::cout << "- " << cam->name() << std::endl;
	}

	if (options.isSet(OptCamera)) {
		camera = cm->get(options[OptCamera]);
		if (!camera) {
			std::cout << "Camera "
				  << std::string(options[OptCamera])
				  << " not found" << std::endl;
			goto out;
		}

		if (camera->acquire()) {
			std::cout << "Failed to acquire camera" << std::endl;
			goto out;
		}

		std::cout << "Using camera " << camera->name() << std::endl;
	}

	if (options.isSet(OptCapture)) {
		if (!camera) {
			std::cout << "Can't capture without a camera"
				  << std::endl;
			ret = EXIT_FAILURE;
			goto out;
		}

		if (options.isSet(OptFile)) {
			if (!options[OptFile].toString().empty())
				writer = new BufferWriter(options[OptFile]);
			else
				writer = new BufferWriter();
		}

		capture();

		if (options.isSet(OptFile)) {
			delete writer;
			writer = nullptr;
		}
	}

	if (camera) {
		camera->release();
		camera.reset();
	}
out:
	delete loop;

	cm->stop();

	return ret;
}
