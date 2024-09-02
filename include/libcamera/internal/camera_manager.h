/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Ideas on Board Oy.
 *
 * Camera manager private data
 */

#pragma once

#include <libcamera/camera_manager.h>

#include <memory>
#include <sys/types.h>
#include <vector>

#include <libcamera/base/class.h>
#include <libcamera/base/mutex.h>
#include <libcamera/base/thread.h>
#include <libcamera/base/thread_annotations.h>

#include "libcamera/internal/process.h"

namespace libcamera {

class Camera;
class DeviceEnumerator;
class IPAManager;
class PipelineHandlerFactoryBase;

class CameraManager::Private : public Extensible::Private, public Thread
{
	LIBCAMERA_DECLARE_PUBLIC(CameraManager)

public:
	Private();

	int start();
	void addCamera(std::shared_ptr<Camera> camera) LIBCAMERA_TSA_EXCLUDES(mutex_);
	void removeCamera(std::shared_ptr<Camera> camera) LIBCAMERA_TSA_EXCLUDES(mutex_);

	IPAManager *ipaManager() const { return ipaManager_.get(); }

protected:
	void run() override;

private:
	int init();
	void createPipelineHandlers();
	void pipelineFactoryMatch(const PipelineHandlerFactoryBase *factory);
	void cleanup() LIBCAMERA_TSA_EXCLUDES(mutex_);

	/*
	 * This mutex protects
	 *
	 * - initialized_ and status_ during initialization
	 * - cameras_ after initialization
	 */
	mutable Mutex mutex_;
	std::vector<std::shared_ptr<Camera>> cameras_ LIBCAMERA_TSA_GUARDED_BY(mutex_);

	ConditionVariable cv_;
	bool initialized_ LIBCAMERA_TSA_GUARDED_BY(mutex_);
	int status_ LIBCAMERA_TSA_GUARDED_BY(mutex_);

	std::unique_ptr<DeviceEnumerator> enumerator_;

	std::unique_ptr<IPAManager> ipaManager_;
	ProcessManager processManager_;
};

} /* namespace libcamera */
