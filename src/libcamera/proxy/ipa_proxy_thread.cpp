/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * ipa_proxy_thread.cpp - Proxy running an Image Processing Algorithm in a thread
 */

#include <memory>

#include <libcamera/ipa/ipa_interface.h>
#include <libcamera/ipa/ipa_module_info.h>

#include "libcamera/internal/ipa_context_wrapper.h"
#include "libcamera/internal/ipa_module.h"
#include "libcamera/internal/ipa_proxy.h"
#include "libcamera/internal/log.h"
#include "libcamera/internal/thread.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAProxy)

class IPAProxyThread : public IPAProxy, public Object
{
public:
	IPAProxyThread(IPAModule *ipam);

	int init(const IPASettings &settings) override;
	int start() override;
	void stop() override;

	void configure(const CameraSensorInfo &sensorInfo,
		       const std::map<unsigned int, IPAStream> &streamConfig,
		       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
		       const IPAOperationData &ipaConfig,
		       IPAOperationData *result) override;
	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;
	void processEvent(const IPAOperationData &event) override;

private:
	void queueFrameAction(unsigned int frame, const IPAOperationData &data);

	/* Helper class to invoke processEvent() in another thread. */
	class ThreadProxy : public Object
	{
	public:
		void setIPA(IPAInterface *ipa)
		{
			ipa_ = ipa;
		}

		int start()
		{
			return ipa_->start();
		}

		void stop()
		{
			ipa_->stop();
		}

		void processEvent(const IPAOperationData &event)
		{
			ipa_->processEvent(event);
		}

	private:
		IPAInterface *ipa_;
	};

	bool running_;
	Thread thread_;
	ThreadProxy proxy_;
	std::unique_ptr<IPAInterface> ipa_;
};

IPAProxyThread::IPAProxyThread(IPAModule *ipam)
	: IPAProxy(ipam), running_(false)
{
	if (!ipam->load())
		return;

	struct ipa_context *ctx = ipam->createContext();
	if (!ctx) {
		LOG(IPAProxy, Error)
			<< "Failed to create IPA context for " << ipam->path();
		return;
	}

	ipa_ = std::make_unique<IPAContextWrapper>(ctx);
	proxy_.setIPA(ipa_.get());

	/*
	 * Proxy the queueFrameAction signal to dispatch it in the caller's
	 * thread.
	 */
	ipa_->queueFrameAction.connect(this, &IPAProxyThread::queueFrameAction);

	valid_ = true;
}

int IPAProxyThread::init(const IPASettings &settings)
{
	int ret = ipa_->init(settings);
	if (ret)
		return ret;

	proxy_.moveToThread(&thread_);

	return 0;
}

int IPAProxyThread::start()
{
	running_ = true;
	thread_.start();

	return proxy_.invokeMethod(&ThreadProxy::start, ConnectionTypeBlocking);
}

void IPAProxyThread::stop()
{
	if (!running_)
		return;

	running_ = false;

	proxy_.invokeMethod(&ThreadProxy::stop, ConnectionTypeBlocking);

	thread_.exit();
	thread_.wait();
}

void IPAProxyThread::configure(const CameraSensorInfo &sensorInfo,
			       const std::map<unsigned int, IPAStream> &streamConfig,
			       const std::map<unsigned int, const ControlInfoMap &> &entityControls,
			       const IPAOperationData &ipaConfig,
			       IPAOperationData *result)
{
	ipa_->configure(sensorInfo, streamConfig, entityControls, ipaConfig,
			result);
}

void IPAProxyThread::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	ipa_->mapBuffers(buffers);
}

void IPAProxyThread::unmapBuffers(const std::vector<unsigned int> &ids)
{
	ipa_->unmapBuffers(ids);
}

void IPAProxyThread::processEvent(const IPAOperationData &event)
{
	if (!running_)
		return;

	/* Dispatch the processEvent() call to the thread. */
	proxy_.invokeMethod(&ThreadProxy::processEvent, ConnectionTypeQueued,
			    event);
}

void IPAProxyThread::queueFrameAction(unsigned int frame, const IPAOperationData &data)
{
	IPAInterface::queueFrameAction.emit(frame, data);
}

REGISTER_IPA_PROXY(IPAProxyThread)

} /* namespace libcamera */
