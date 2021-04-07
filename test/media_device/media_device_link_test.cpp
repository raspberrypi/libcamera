/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * media_device_link_test.cpp - Tests link handling on VIMC media device
 */

#include <iostream>

#include "media_device_test.h"

using namespace libcamera;
using namespace std;

/*
 * This link test requires a vimc device in order to exercise the
 * MediaObject link handling API on a graph with a predetermined topology.
 *
 * vimc is a Media Controller kernel driver that creates virtual devices.
 * From a userspace point of view they appear as normal media controller
 * devices, but are not backed by any particular piece of hardware. They can
 * thus be used for testing purpose without depending on a particular hardware
 * platform.
 *
 * If no vimc device is found (most likely because the vimc driver is not
 * loaded) the test is skipped.
 */

class MediaDeviceLinkTest : public MediaDeviceTest
{
	int init()
	{
		int ret = MediaDeviceTest::init();
		if (ret)
			return ret;

		if (!media_->acquire()) {
			cerr << "Unable to acquire media device "
			     << media_->deviceNode() << endl;
			return TestFail;
		}

		return TestPass;
	}

	int run()
	{
		/*
		 * First of all disable all links in the media graph to
		 * ensure we start from a known state.
		 */
		if (media_->disableLinks()) {
			cerr << "Failed to disable all links in the media graph";
			return TestFail;
		}

		/*
		 * Test if link can be consistently retrieved through the
		 * different functions the media device offers.
		 */
		string linkName("'Debayer A':[1] -> 'Scaler':[0]'");
		MediaLink *link = media_->link("Debayer A", 1, "Scaler", 0);
		if (!link) {
			cerr << "Unable to find link: " << linkName
			     << " using lookup by name" << endl;
			return TestFail;
		}

		MediaEntity *source = media_->getEntityByName("Debayer A");
		if (!source) {
			cerr << "Unable to find entity: 'Debayer A'" << endl;
			return TestFail;
		}

		MediaEntity *sink = media_->getEntityByName("Scaler");
		if (!sink) {
			cerr << "Unable to find entity: 'Scaler'" << endl;
			return TestFail;
		}

		MediaLink *link2 = media_->link(source, 1, sink, 0);
		if (!link2) {
			cerr << "Unable to find link: " << linkName
			     << " using lookup by entity" << endl;
			return TestFail;
		}

		if (link != link2) {
			cerr << "Link lookup by name and by entity don't match"
			     << endl;
			return TestFail;
		}

		link2 = media_->link(source->getPadByIndex(1),
				   sink->getPadByIndex(0));
		if (!link2) {
			cerr << "Unable to find link: " << linkName
			     << " using lookup by pad" << endl;
			return TestFail;
		}

		if (link != link2) {
			cerr << "Link lookup by name and by pad don't match"
			     << endl;
			return TestFail;
		}

		/* After reset the link shall not be enabled. */
		if (link->flags() & MEDIA_LNK_FL_ENABLED) {
			cerr << "Link " << linkName
			     << " should not be enabled after a device reset"
			     << endl;
			return TestFail;
		}

		/* Enable the link and test if enabling was successful. */
		if (link->setEnabled(true)) {
			cerr << "Failed to enable link: " << linkName
			     << endl;
			return TestFail;
		}

		if (!(link->flags() & MEDIA_LNK_FL_ENABLED)) {
			cerr << "Link " << linkName
			     << " was enabled but it is reported as disabled"
			     << endl;
			return TestFail;
		}

		/* Disable the link and test if disabling was successful. */
		if (link->setEnabled(false)) {
			cerr << "Failed to disable link: " << linkName
			     << endl;
			return TestFail;
		}

		if (link->flags() & MEDIA_LNK_FL_ENABLED) {
			cerr << "Link " << linkName
			     << " was disabled but it is reported as enabled"
			     << endl;
			return TestFail;
		}

		/* Try to get a non existing link. */
		linkName = "'Sensor A':[1] -> 'Scaler':[0]";
		link = media_->link("Sensor A", 1, "Scaler", 0);
		if (link) {
			cerr << "Link lookup for " << linkName
			     << " succeeded but link does not exist"
			     << endl;
			return TestFail;
		}

		/* Now get an immutable link and try to disable it. */
		linkName = "'Sensor A':[0] -> 'Raw Capture 0':[0]";
		link = media_->link("Sensor A", 0, "Raw Capture 0", 0);
		if (!link) {
			cerr << "Unable to find link: " << linkName
			     << " using lookup by name" << endl;
			return TestFail;
		}

		if (!(link->flags() & MEDIA_LNK_FL_IMMUTABLE)) {
			cerr << "Link " << linkName
			     << " should be 'IMMUTABLE'" << endl;
			return TestFail;
		}

		/* Disabling an immutable link shall fail. */
		if (!link->setEnabled(false)) {
			cerr << "Disabling immutable link " << linkName
			     << " succeeded but should have failed" << endl;
			return TestFail;
		}

		/*
		 * Enable an disabled link, and verify it is disabled again
		 * after disabling all links in the media graph.
		 */
		linkName = "'Debayer B':[1] -> 'Scaler':[0]'";
		link = media_->link("Debayer B", 1, "Scaler", 0);
		if (!link) {
			cerr << "Unable to find link: " << linkName
			     << " using lookup by name" << endl;
			return TestFail;
		}

		if (link->setEnabled(true)) {
			cerr << "Failed to enable link: " << linkName
			     << endl;
			return TestFail;
		}

		if (!(link->flags() & MEDIA_LNK_FL_ENABLED)) {
			cerr << "Link " << linkName
			     << " was enabled but it is reported as disabled"
			     << endl;
			return TestFail;
		}

		if (media_->disableLinks()) {
			cerr << "Failed to disable all links in the media graph";
			return TestFail;
		}

		if (link->flags() & MEDIA_LNK_FL_ENABLED) {
			cerr << "All links in the media graph have been disabled"
			     << " but link " << linkName
			     << " is still reported as enabled"  << endl;
			return TestFail;
		}

		return 0;
	}

	void cleanup()
	{
		media_->release();
	}
};

TEST_REGISTER(MediaDeviceLinkTest)
