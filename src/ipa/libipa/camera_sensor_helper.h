/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_helper.h - Helper class that performs sensor-specific parameter computations
 */

#pragma once

#include <stdint.h>

#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/class.h>

namespace libcamera {

namespace ipa {

class CameraSensorHelper
{
public:
	CameraSensorHelper() = default;
	virtual ~CameraSensorHelper() = default;

	virtual uint32_t gainCode(double gain) const;
	virtual double gain(uint32_t gainCode) const;

protected:
	enum AnalogueGainType {
		AnalogueGainLinear,
		AnalogueGainExponential,
	};

	struct AnalogueGainLinearConstants {
		int16_t m0;
		int16_t c0;
		int16_t m1;
		int16_t c1;
	};

	struct AnalogueGainExpConstants {
		double a;
		double m;
	};

	union AnalogueGainConstants {
		AnalogueGainLinearConstants linear;
		AnalogueGainExpConstants exp;
	};

	AnalogueGainType gainType_;
	AnalogueGainConstants gainConstants_;

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorHelper)
};

class CameraSensorHelperFactoryBase
{
public:
	CameraSensorHelperFactoryBase(const std::string name);
	virtual ~CameraSensorHelperFactoryBase() = default;

	static std::unique_ptr<CameraSensorHelper> create(const std::string &name);

	static std::vector<CameraSensorHelperFactoryBase *> &factories();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorHelperFactoryBase)

	static void registerType(CameraSensorHelperFactoryBase *factory);

	virtual std::unique_ptr<CameraSensorHelper> createInstance() const = 0;

	std::string name_;
};

template<typename _Helper>
class CameraSensorHelperFactory final : public CameraSensorHelperFactoryBase
{
public:
	CameraSensorHelperFactory(const char *name)
		: CameraSensorHelperFactoryBase(name)
	{
	}

private:
	std::unique_ptr<CameraSensorHelper> createInstance() const
	{
		return std::make_unique<_Helper>();
	}
};

#define REGISTER_CAMERA_SENSOR_HELPER(name, helper) \
static CameraSensorHelperFactory<helper> global_##helper##Factory(name);

} /* namespace ipa */

} /* namespace libcamera */
