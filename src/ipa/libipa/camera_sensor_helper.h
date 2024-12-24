/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Helper class that performs sensor-specific parameter computations
 */

#pragma once

#include <memory>
#include <optional>
#include <stdint.h>
#include <string>
#include <variant>
#include <vector>

#include <libcamera/base/class.h>

namespace libcamera {

namespace ipa {

class CameraSensorHelper
{
public:
	CameraSensorHelper() = default;
	virtual ~CameraSensorHelper() = default;

	std::optional<int16_t> blackLevel() const { return blackLevel_; }
	virtual uint32_t gainCode(double gain) const;
	virtual double gain(uint32_t gainCode) const;

protected:
	struct AnalogueGainLinear {
		int16_t m0;
		int16_t c0;
		int16_t m1;
		int16_t c1;
	};

	struct AnalogueGainExp {
		double a;
		double m;
	};

	std::optional<int16_t> blackLevel_;
	std::variant<std::monostate, AnalogueGainLinear, AnalogueGainExp> gain_;

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
	std::unique_ptr<CameraSensorHelper> createInstance() const override
	{
		return std::make_unique<_Helper>();
	}
};

#define REGISTER_CAMERA_SENSOR_HELPER(name, helper) \
static CameraSensorHelperFactory<helper> global_##helper##Factory(name);

} /* namespace ipa */

} /* namespace libcamera */
