/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * main.cpp - cam - The libcamera swiss army knife
 */

#include <iomanip>
#include <iostream>
#include <map>
#include <signal.h>
#include <string.h>

#include <libcamera/libcamera.h>

#include "buffer_writer.h"
#include "event_loop.h"
#include "options.h"

using namespace libcamera;

OptionsParser::Options options;
std::shared_ptr<Camera> camera;
EventLoop *loop;
BufferWriter *writer;

enum {
	OptCamera = 'c',
	OptCapture = 'C',
	OptFile = 'F',
	OptFormat = 'f',
	OptHelp = 'h',
	OptList = 'l',
};

void signalHandler(int signal)
{
	std::cout << "Exiting" << std::endl;
	loop->exit();
}

static int parseOptions(int argc, char *argv[])
{
	KeyValueParser formatKeyValue;
	formatKeyValue.addOption("width", OptionInteger, "Width in pixels",
				 ArgumentRequired);
	formatKeyValue.addOption("height", OptionInteger, "Height in pixels",
				 ArgumentRequired);
	formatKeyValue.addOption("pixelformat", OptionInteger, "Pixel format",
				 ArgumentRequired);

	OptionsParser parser;
	parser.addOption(OptCamera, OptionString,
			 "Specify which camera to operate on", "camera",
			 ArgumentRequired, "camera");
	parser.addOption(OptCapture, OptionNone,
			 "Capture until interrupted by user", "capture");
	parser.addOption(OptFile, OptionString,
			 "Write captured frames to disk\n"
			 "The first '#' character in the file name is expanded to the frame sequence number.\n"
			 "The default file name is 'frame-#.bin'.",
			 "file", ArgumentOptional, "filename");
	parser.addOption(OptFormat, &formatKeyValue,
			 "Set format of the camera's first stream", "format");
	parser.addOption(OptHelp, OptionNone, "Display this help message",
			 "help");
	parser.addOption(OptList, OptionNone, "List all cameras", "list");

	options = parser.parse(argc, argv);
	if (!options.valid())
		return -EINVAL;

	if (argc == 1 || options.isSet(OptHelp)) {
		parser.usage();
		return 1;
	}

	return 0;
}

static int configureStreams(Camera *camera, std::vector<Stream *> &streams)
{
	KeyValueParser::Options format = options[OptFormat];
	Stream *id = streams.front();

	std::map<Stream *, StreamConfiguration> config =
		camera->streamConfiguration(streams);

	if (options.isSet(OptFormat)) {
		if (format.isSet("width"))
			config[id].width = format["width"];

		if (format.isSet("height"))
			config[id].height = format["height"];

		/* TODO: Translate 4CC string to ID. */
		if (format.isSet("pixelformat"))
			config[id].pixelFormat = format["pixelformat"];
	}

	return camera->configureStreams(config);
}

static void requestComplete(Request *request, const std::map<Stream *, Buffer *> &buffers)
{
	static uint64_t last = 0;

	Buffer *buffer = buffers.begin()->second;

	double fps = buffer->timestamp() - last;
	fps = last && fps ? 1000000000.0 / fps : 0.0;
	last = buffer->timestamp();

	std::cout << "seq: " << std::setw(6) << std::setfill('0') << buffer->sequence()
		  << " buf: " << buffer->index()
		  << " bytesused: " << buffer->bytesused()
		  << " timestamp: " << buffer->timestamp()
		  << " fps: " << std::fixed << std::setprecision(2) << fps
		  << std::endl;

	if (writer)
		writer->write(buffer);

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
	int ret;

	std::vector<Stream *> streams = camera->streams();

	ret = configureStreams(camera.get(), streams);
	if (ret < 0) {
		std::cout << "Failed to configure camera" << std::endl;
		return ret;
	}

	Stream *stream = streams.front();

	ret = camera->allocateBuffers();
	if (ret) {
		std::cerr << "Failed to allocate buffers"
			  << std::endl;
		return ret;
	}

	camera->requestCompleted.connect(requestComplete);

	BufferPool &pool = stream->bufferPool();

	for (Buffer &buffer : pool.buffers()) {
		Request *request = camera->createRequest();
		if (!request) {
			std::cerr << "Can't create request" << std::endl;
			ret = -ENOMEM;
			goto out;
		}

		std::map<Stream *, Buffer *> map;
		map[stream] = &buffer;
		ret = request->setBuffers(map);
		if (ret < 0) {
			std::cerr << "Can't set buffers for request" << std::endl;
			goto out;
		}

		ret = camera->queueRequest(request);
		if (ret < 0) {
			std::cerr << "Can't queue request" << std::endl;
			goto out;
		}
	}

	std::cout << "Capture until user interrupts by SIGINT" << std::endl;

	ret = camera->start();
	if (ret) {
		std::cout << "Failed to start capture" << std::endl;
		goto out;
	}

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
		return EXIT_FAILURE;

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
			std::cout << "Camera " << options[OptCamera]
				  << " not found" << std::endl;
			goto out;
		}

		const std::vector<Stream *> &streams = camera->streams();
		if (streams.size() != 1) {
			std::cout << "Camera has " << streams.size()
				  << " streams, only 1 is supported"
				  << std::endl;
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
