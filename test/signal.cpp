/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * signal.cpp - Signal test
 */

#include <iostream>
#include <string.h>

#include <libcamera/base/object.h>
#include <libcamera/base/signal.h>

#include "test.h"

using namespace std;
using namespace libcamera;

static int valueStatic_ = 0;

static void slotStatic(int value)
{
	valueStatic_ = value;
}

static int slotStaticReturn()
{
	return 0;
}

class SlotObject : public Object
{
public:
	void slot()
	{
		valueStatic_ = 1;
	}
};

class BaseClass
{
public:
	/*
	 * A virtual function is required in the base class, otherwise the
	 * compiler will always store Object before BaseClass in memory.
	 */
	virtual ~BaseClass()
	{
	}

	unsigned int data_[32];
};

class SlotMulti : public BaseClass, public Object
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

	int slotReturn()
	{
		return 0;
	}

	int init()
	{
		return 0;
	}

	int run()
	{
		/* ----------------- Signal -> !Object tests ---------------- */

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
		 * Test connecting to slots that return a value. This targets
		 * compilation, there's no need to check runtime results.
		 */
		signalVoid_.connect(slotStaticReturn);
		signalVoid_.connect(this, &SignalTest::slotReturn);

		/* Test signal connection to a lambda. */
		int value = 0;
		signalInt_.connect(this, [&](int v) { value = v; });
		signalInt_.emit(42);

		if (value != 42) {
			cout << "Signal connection to lambda failed" << endl;
			return TestFail;
		}

		signalInt_.disconnect(this);
		signalInt_.emit(0);

		if (value != 42) {
			cout << "Signal disconnection from lambda failed" << endl;
			return TestFail;
		}

		/* ----------------- Signal -> Object tests ----------------- */

		/*
		 * Test automatic disconnection on object deletion. Connect two
		 * signals to ensure all instances are disconnected.
		 */
		signalVoid_.disconnect();
		signalVoid2_.disconnect();

		SlotObject *slotObject = new SlotObject();
		signalVoid_.connect(slotObject, &SlotObject::slot);
		signalVoid2_.connect(slotObject, &SlotObject::slot);
		delete slotObject;
		valueStatic_ = 0;
		signalVoid_.emit();
		signalVoid2_.emit();
		if (valueStatic_ != 0) {
			cout << "Signal disconnection on object deletion test failed" << endl;
			return TestFail;
		}

		/*
		 * Test that signal deletion disconnects objects. This shall
		 * not generate any valgrind warning.
		 */
		Signal<> *dynamicSignal = new Signal<>();
		slotObject = new SlotObject();
		dynamicSignal->connect(slotObject, &SlotObject::slot);
		delete dynamicSignal;
		delete slotObject;

		/*
		 * Test that signal manual disconnection from Object removes
		 * the signal for the object. This shall not generate any
		 * valgrind warning.
		 */
		dynamicSignal = new Signal<>();
		slotObject = new SlotObject();
		dynamicSignal->connect(slotObject, &SlotObject::slot);
		dynamicSignal->disconnect(slotObject);
		delete dynamicSignal;
		delete slotObject;

		/*
		 * Test that signal manual disconnection from all slots removes
		 * the signal for the object. This shall not generate any
		 * valgrind warning.
		 */
		dynamicSignal = new Signal<>();
		slotObject = new SlotObject();
		dynamicSignal->connect(slotObject, &SlotObject::slot);
		dynamicSignal->disconnect();
		delete dynamicSignal;
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

		/* Test signal connection to a lambda. */
		slotObject = new SlotObject();
		value = 0;
		signalInt_.connect(slotObject, [&](int v) { value = v; });
		signalInt_.emit(42);

		if (value != 42) {
			cout << "Signal connection to Object lambda failed" << endl;
			return TestFail;
		}

		signalInt_.disconnect(slotObject);
		signalInt_.emit(0);

		if (value != 42) {
			cout << "Signal disconnection from Object lambda failed" << endl;
			return TestFail;
		}

		delete slotObject;

		/* --------- Signal -> Object (multiple inheritance) -------- */

		/*
		 * Test automatic disconnection on object deletion. Connect two
		 * signals to ensure all instances are disconnected.
		 */
		signalVoid_.disconnect();
		signalVoid2_.disconnect();

		SlotMulti *slotMulti = new SlotMulti();
		signalVoid_.connect(slotMulti, &SlotMulti::slot);
		signalVoid2_.connect(slotMulti, &SlotMulti::slot);
		delete slotMulti;
		valueStatic_ = 0;
		signalVoid_.emit();
		signalVoid2_.emit();
		if (valueStatic_ != 0) {
			cout << "Signal disconnection on object deletion test failed" << endl;
			return TestFail;
		}

		/*
		 * Test that signal deletion disconnects objects. This shall
		 * not generate any valgrind warning.
		 */
		dynamicSignal = new Signal<>();
		slotMulti = new SlotMulti();
		dynamicSignal->connect(slotMulti, &SlotMulti::slot);
		delete dynamicSignal;
		delete slotMulti;

		/* Exercise the Object slot code paths. */
		slotMulti = new SlotMulti();
		signalVoid_.connect(slotMulti, &SlotMulti::slot);
		valueStatic_ = 0;
		signalVoid_.emit();
		if (valueStatic_ == 0) {
			cout << "Signal delivery for Object test failed" << endl;
			return TestFail;
		}

		delete slotMulti;

		return TestPass;
	}

	void cleanup()
	{
	}

private:
	Signal<> signalVoid_;
	Signal<> signalVoid2_;
	Signal<int> signalInt_;
	Signal<int, const std::string &> signalMultiArgs_;

	bool called_;
	int values_[3];
	std::string name_;
};

TEST_REGISTER(SignalTest)
