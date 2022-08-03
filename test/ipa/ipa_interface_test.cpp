/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2019, Google Inc.
 *
 * ipa_interface_test.cpp - Test the IPA interface
 */

#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <libcamera/ipa/vimc_ipa_proxy.h>

#include <libcamera/base/event_dispatcher.h>
#include <libcamera/base/event_notifier.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/timer.h>

#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/process.h"

#include "test.h"

using namespace libcamera;
using namespace std;
using namespace std::chrono_literals;

class IPAInterfaceTest : public Test, public Object
{
public:
	IPAInterfaceTest()
		: trace_(ipa::vimc::IPAOperationNone), notifier_(nullptr), fd_(-1)
	{
	}

	~IPAInterfaceTest()
	{
		delete notifier_;
		ipa_.reset();
		ipaManager_.reset();
	}

protected:
	int init() override
	{
		ipaManager_ = make_unique<IPAManager>();

		/* Create a pipeline handler for vimc. */
		const std::vector<PipelineHandlerFactoryBase *> &factories =
			PipelineHandlerFactoryBase::factories();
		for (const PipelineHandlerFactoryBase *factory : factories) {
			if (factory->name() == "PipelineHandlerVimc") {
				pipe_ = factory->create(nullptr);
				break;
			}
		}

		if (!pipe_) {
			cerr << "Vimc pipeline not found" << endl;
			return TestPass;
		}

		/* Create and open the communication FIFO. */
		int ret = mkfifo(ipa::vimc::VimcIPAFIFOPath.c_str(), S_IRUSR | S_IWUSR);
		if (ret) {
			ret = errno;
			cerr << "Failed to create IPA test FIFO at '"
			     << ipa::vimc::VimcIPAFIFOPath << "': " << strerror(ret)
			     << endl;
			return TestFail;
		}

		ret = open(ipa::vimc::VimcIPAFIFOPath.c_str(), O_RDONLY | O_NONBLOCK);
		if (ret < 0) {
			ret = errno;
			cerr << "Failed to open IPA test FIFO at '"
			     << ipa::vimc::VimcIPAFIFOPath << "': " << strerror(ret)
			     << endl;
			unlink(ipa::vimc::VimcIPAFIFOPath.c_str());
			return TestFail;
		}
		fd_ = ret;

		notifier_ = new EventNotifier(fd_, EventNotifier::Read, this);
		notifier_->activated.connect(this, &IPAInterfaceTest::readTrace);

		return TestPass;
	}

	int run() override
	{
		EventDispatcher *dispatcher = thread()->eventDispatcher();
		Timer timer;

		ipa_ = IPAManager::createIPA<ipa::vimc::IPAProxyVimc>(pipe_.get(), 0, 0);
		if (!ipa_) {
			cerr << "Failed to create VIMC IPA interface" << endl;
			return TestFail;
		}

		/* Test initialization of IPA module. */
		std::string conf = ipa_->configurationFile("vimc.conf");
		Flags<ipa::vimc::TestFlag> inFlags;
		Flags<ipa::vimc::TestFlag> outFlags;
		int ret = ipa_->init(IPASettings{ conf, "vimc" },
				     ipa::vimc::IPAOperationInit,
				     inFlags, &outFlags);
		if (ret < 0) {
			cerr << "IPA interface init() failed" << endl;
			return TestFail;
		}

		timer.start(1000ms);
		while (timer.isRunning() && trace_ != ipa::vimc::IPAOperationInit)
			dispatcher->processEvents();

		if (trace_ != ipa::vimc::IPAOperationInit) {
			cerr << "Failed to test IPA initialization sequence"
			     << endl;
			return TestFail;
		}

		/* Test start of IPA module. */
		ipa_->start();
		timer.start(1000ms);
		while (timer.isRunning() && trace_ != ipa::vimc::IPAOperationStart)
			dispatcher->processEvents();

		if (trace_ != ipa::vimc::IPAOperationStart) {
			cerr << "Failed to test IPA start sequence" << endl;
			return TestFail;
		}

		/* Test stop of IPA module. */
		ipa_->stop();
		timer.start(1000ms);
		while (timer.isRunning() && trace_ != ipa::vimc::IPAOperationStop)
			dispatcher->processEvents();

		if (trace_ != ipa::vimc::IPAOperationStop) {
			cerr << "Failed to test IPA stop sequence" << endl;
			return TestFail;
		}

		return TestPass;
	}

	void cleanup() override
	{
		close(fd_);
		unlink(ipa::vimc::VimcIPAFIFOPath.c_str());
	}

private:
	void readTrace()
	{
		ssize_t s = read(notifier_->fd(), &trace_, sizeof(trace_));
		if (s < 0) {
			int ret = errno;
			cerr << "Failed to read from IPA test FIFO at '"
			     << ipa::vimc::VimcIPAFIFOPath << "': " << strerror(ret)
			     << endl;
			trace_ = ipa::vimc::IPAOperationNone;
		}
	}

	ProcessManager processManager_;

	std::shared_ptr<PipelineHandler> pipe_;
	std::unique_ptr<ipa::vimc::IPAProxyVimc> ipa_;
	std::unique_ptr<IPAManager> ipaManager_;
	enum ipa::vimc::IPAOperationCode trace_;
	EventNotifier *notifier_;
	int fd_;
};

TEST_REGISTER(IPAInterfaceTest)
