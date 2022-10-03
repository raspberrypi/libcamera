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

class CameraSensorHelperFactory
{
public:
	CameraSensorHelperFactory(const std::string name);
	virtual ~CameraSensorHelperFactory() = default;

	static std::unique_ptr<CameraSensorHelper> create(const std::string &name);

	static std::vector<CameraSensorHelperFactory *> &factories();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(CameraSensorHelperFactory)

	static void registerType(CameraSensorHelperFactory *factory);

	virtual std::unique_ptr<CameraSensorHelper> createInstance() const = 0;

	std::string name_;
};

#define REGISTER_CAMERA_SENSOR_HELPER(name, helper)		\
class helper##Factory final : public CameraSensorHelperFactory	\
{								\
public: 							\
	helper##Factory() : CameraSensorHelperFactory(name) {}	\
								\
private:							\
	std::unique_ptr<CameraSensorHelper> createInstance() const \
	{							\
		return std::make_unique<helper>();		\
	}							\
};								\
static helper##Factory global_##helper##Factory;

} /* namespace ipa */

} /* namespace libcamera */
