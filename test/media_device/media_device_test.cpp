/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2018, Google Inc.
 *
 * media_device_test.cpp - Tests for the media device class.
 *
 * Test library for the media device class.
 */
#include <iostream>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "media_device.h"

#include "test.h"

using namespace libcamera;
using namespace std;

/*
 * MediaDeviceTest object: runs a sequence of tests on all media
 * devices found in the system.
 *
 * If no accessible media device is found, the test is skipped.
 */
class MediaDeviceTest : public Test
{
public:
	MediaDeviceTest() { }
	~MediaDeviceTest() { }

protected:
	int init() { return 0; }
	int run();
	void cleanup() { }

private:
	int testMediaDevice(string devnode);

	void printMediaGraph(const MediaDevice &media, ostream &os);
	void printLinkFlags(const MediaLink *link, ostream &os);
	void printNode(const MediaPad *pad, ostream &os);
};

void MediaDeviceTest::printNode(const MediaPad *pad, ostream &os)
{
	const MediaEntity *entity = pad->entity();

	os << "\"" << entity->name() << "\"["
	   << pad->index() << "]";
}

void MediaDeviceTest::printLinkFlags(const MediaLink *link, ostream &os)
{
	unsigned int flags = link->flags();

	os << " [";
	if (flags) {
		os << (flags & MEDIA_LNK_FL_ENABLED ? "ENABLED," : "")
		   << (flags & MEDIA_LNK_FL_IMMUTABLE ? "IMMUTABLE" : "");
	}
	os  << "]\n";
}

/*
 * For each entity in the media graph, printout links directed to its sinks
 * and source pads.
 */
void MediaDeviceTest::printMediaGraph(const MediaDevice &media, ostream &os)
{
	os << "\n" << media.driver() << " - " << media.devnode() << "\n\n";

	for (auto const &entity : media.entities()) {
		os << "\"" << entity->name() << "\"\n";

		for (auto const &sink : entity->pads()) {
			if (!(sink->flags() & MEDIA_PAD_FL_SINK))
				continue;

			os << "  [" << sink->index() << "]" << ": Sink\n";
			for (auto const &link : sink->links()) {
				os << "\t";
				printNode(sink, os);
				os << " <- ";
				printNode(link->source(), os);
				printLinkFlags(link, os);
			}
			os << "\n";
		}

		for (auto const &source : entity->pads()) {
			if (!(source->flags() & MEDIA_PAD_FL_SOURCE))
				continue;

			os << "  [" << source->index() << "]" << ": Source\n";
			for (auto const &link : source->links()) {
				os << "\t";
				printNode(source, os);
				os << " -> ";
				printNode(link->sink(), os);
				printLinkFlags(link, os);
			}
			os << "\n";
		}
	}

	os.flush();
}

/* Test a single media device. */
int MediaDeviceTest::testMediaDevice(const string devnode)
{
	MediaDevice dev(devnode);
	int ret;

	/* Fuzzy open/close sequence. */
	ret = dev.open();
	if (ret)
		return ret;

	ret = dev.open();
	if (!ret)
		return ret;

	dev.close();

	ret = dev.open();
	if (ret)
		return ret;

	ret = dev.populate();
	if (ret)
		return ret;

	/* Run tests in sequence. */
	printMediaGraph(dev, cerr);
	/* TODO: add more tests here. */

	dev.close();

	return 0;
}

/* Run tests on all media devices. */
#define MAX_MEDIA_DEV 256
int MediaDeviceTest::run()
{
	const string devnode("/dev/media");
	unsigned int i;
	int ret = 77; /* skip test exit code */

	/*
	 * Run the test sequence on all media device found in the
	 * system, if any.
	 */
	for (i = 0; i < MAX_MEDIA_DEV; i++) {
		string mediadev = devnode + to_string(i);
		struct stat pstat = { };

		if (stat(mediadev.c_str(), &pstat))
			continue;

		ret = testMediaDevice(mediadev);
		if (ret)
			return ret;

	}

	return ret;
}

TEST_REGISTER(MediaDeviceTest);
