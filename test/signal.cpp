/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * signal.cpp - Signal test
 */

#include <iostream>
#include <string.h>

#include <libcamera/object.h>
#include <libcamera/signal.h>

#include "test.h"

using namespace std;
using namespace libcamera;

static int valueStatic_ = 0;

static void slotStatic(int value)
{
	valueStatic_ = value;
}

class SlotObject : public Object
{
public:
	void slot()
	{
		valueStatic_ = 1;
	}
};

class SignalTest : public Test
{
protected:
	void slotVoid()
	{
		called_ = true;
	}

	void slotDisconnect()
	{
		called_ = true;
		signalVoid_.disconnect(this, &SignalTest::slotDisconnect);
	}

	void slotInteger1(int value)
	{
		values_[0] = value;
	}

	void slotInteger2(int value)
	{
		values_[1] = value;
	}

	void slotMultiArgs(int value, const std::string &name)
	{
		values_[2] = value;
		name_ = name;
	}

	int init()
	{
		return 0;
	}

	int run()
	{
		/* Test signal emission and reception. */
		called_ = false;
		signalVoid_.connect(this, &SignalTest::slotVoid);
		signalVoid_.emit();

		if (!called_) {
			cout << "Signal emission test failed" << endl;
			return TestFail;
		}

		/* Test signal with parameters. */
		values_[2] = 0;
		name_.clear();
		signalMultiArgs_.connect(this, &SignalTest::slotMultiArgs);
		signalMultiArgs_.emit(42, "H2G2");

		if (values_[2] != 42 || name_ != "H2G2") {
			cout << "Signal parameters test failed" << endl;
			return TestFail;
		}

		/* Test signal connected to multiple slots. */
		memset(values_, 0, sizeof(values_));
		valueStatic_ = 0;
		signalInt_.connect(this, &SignalTest::slotInteger1);
		signalInt_.connect(this, &SignalTest::slotInteger2);
		signalInt_.connect(&slotStatic);
		signalInt_.emit(42);

		if (values_[0] != 42 || values_[1] != 42 || values_[2] != 0 ||
		    valueStatic_ != 42) {
			cout << "Signal multi slot test failed" << endl;
			return TestFail;
		}

		/* Test disconnection of a single slot. */
		memset(values_, 0, sizeof(values_));
		signalInt_.disconnect(this, &SignalTest::slotInteger2);
		signalInt_.emit(42);

		if (values_[0] != 42 || values_[1] != 0 || values_[2] != 0) {
			cout << "Signal slot disconnection test failed" << endl;
			return TestFail;
		}

		/* Test disconnection of a whole object. */
		memset(values_, 0, sizeof(values_));
		signalInt_.disconnect(this);
		signalInt_.emit(42);

		if (values_[0] != 0 || values_[1] != 0 || values_[2] != 0) {
			cout << "Signal object disconnection test failed" << endl;
			return TestFail;
		}

		/* Test disconnection of a whole signal. */
		memset(values_, 0, sizeof(values_));
		signalInt_.connect(this, &SignalTest::slotInteger1);
		signalInt_.connect(this, &SignalTest::slotInteger2);
		signalInt_.disconnect();
		signalInt_.emit(42);

		if (values_[0] != 0 || values_[1] != 0 || values_[2] != 0) {
			cout << "Signal object disconnection test failed" << endl;
			return TestFail;
		}

		/* Test disconnection from slot. */
		signalVoid_.disconnect();
		signalVoid_.connect(this, &SignalTest::slotDisconnect);

		signalVoid_.emit();
		called_ = false;
		signalVoid_.emit();

		if (called_) {
			cout << "Signal disconnection from slot test failed" << endl;
			return TestFail;
		}

		/*
		 * Test automatic disconnection on object deletion. Connect the
		 * slot twice to ensure all instances are disconnected.
		 */
		signalVoid_.disconnect();

		SlotObject *slotObject = new SlotObject();
		signalVoid_.connect(slotObject, &SlotObject::slot);
		signalVoid_.connect(slotObject, &SlotObject::slot);
		delete slotObject;
		valueStatic_ = 0;
		signalVoid_.emit();
		if (valueStatic_ != 0) {
			cout << "Signal disconnection on object deletion test failed" << endl;
			return TestFail;
		}

		/*
		 * Test that signal deletion disconnects objects. This shall
		 * not generate any valgrind warning.
		 */
		Signal<> *signal = new Signal<>();
		slotObject = new SlotObject();
		signal->connect(slotObject, &SlotObject::slot);
		delete signal;
		delete slotObject;

		/* Exercise the Object slot code paths. */
		slotObject = new SlotObject();
		signalVoid_.connect(slotObject, &SlotObject::slot);
		valueStatic_ = 0;
		signalVoid_.emit();
		if (valueStatic_ == 0) {
			cout << "Signal delivery for Object test failed" << endl;
			return TestFail;
		}

		delete slotObject;

		return TestPass;
	}

	void cleanup()
	{
	}

private:
	Signal<> signalVoid_;
	Signal<int> signalInt_;
	Signal<int, const std::string &> signalMultiArgs_;

	bool called_;
	int values_[3];
	std::string name_;
};

TEST_REGISTER(SignalTest)
