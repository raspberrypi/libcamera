/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_helper.cpp - Helper class that performs sensor-specific
 * parameter computations
 */
#include "camera_sensor_helper.h"

#include <cmath>

#include <libcamera/base/log.h>

/**
 * \file camera_sensor_helper.h
 * \brief Helper class that performs sensor-specific parameter computations
 *
 * Computation of sensor configuration parameters is a sensor specific
 * operation. Each CameraHelper derived class computes the value of
 * configuration parameters, for example the analogue gain value, using
 * sensor-specific functions and constants.
 *
 * Every subclass of CameraSensorHelper shall be registered with libipa using
 * the REGISTER_CAMERA_SENSOR_HELPER() macro.
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(CameraSensorHelper)

namespace ipa {

/**
 * \class CameraSensorHelper
 * \brief Base class for computing sensor tuning parameters using
 * sensor-specific constants
 *
 * Instances derived from CameraSensorHelper class are sensor-specific.
 * Each supported sensor will have an associated base class defined.
 */

/**
 * \brief Construct a CameraSensorHelper instance
 *
 * CameraSensorHelper derived class instances shall never be constructed
 * manually but always through the CameraSensorHelperFactoryBase::create()
 * function.
 */

/**
 * \brief Compute gain code from the analogue gain absolute value
 * \param[in] gain The real gain to pass
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations.
 *
 * \return The gain code to pass to V4L2
 */
uint32_t CameraSensorHelper::gainCode(double gain) const
{
	const AnalogueGainConstants &k = gainConstants_;

	switch (gainType_) {
	case AnalogueGainLinear:
		ASSERT(k.linear.m0 == 0 || k.linear.m1 == 0);

		return (k.linear.c0 - k.linear.c1 * gain) /
		       (k.linear.m1 * gain - k.linear.m0);

	case AnalogueGainExponential:
		ASSERT(k.exp.a != 0 && k.exp.m != 0);

		return std::log2(gain / k.exp.a) / k.exp.m;

	default:
		ASSERT(false);
		return 0;
	}
}

/**
 * \brief Compute the real gain from the V4L2 subdev control gain code
 * \param[in] gainCode The V4L2 subdev control gain
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations. It is the counterpart of the function
 * CameraSensorHelper::gainCode.
 *
 * \return The real gain
 */
double CameraSensorHelper::gain(uint32_t gainCode) const
{
	const AnalogueGainConstants &k = gainConstants_;
	double gain = static_cast<double>(gainCode);

	switch (gainType_) {
	case AnalogueGainLinear:
		ASSERT(k.linear.m0 == 0 || k.linear.m1 == 0);

		return (k.linear.m0 * gain + k.linear.c0) /
		       (k.linear.m1 * gain + k.linear.c1);

	case AnalogueGainExponential:
		ASSERT(k.exp.a != 0 && k.exp.m != 0);

		return k.exp.a * std::exp2(k.exp.m * gain);

	default:
		ASSERT(false);
		return 0.0;
	}
}

/**
 * \enum CameraSensorHelper::AnalogueGainType
 * \brief The gain calculation modes as defined by the MIPI CCS
 *
 * Describes the image sensor analogue gain capabilities.
 * Two modes are possible, depending on the sensor: Linear and Exponential.
 */

/**
 * \var CameraSensorHelper::AnalogueGainLinear
 * \brief Gain is computed using linear gain estimation
 *
 * The relationship between the integer gain parameter and the resulting gain
 * multiplier is given by the following equation:
 *
 * \f$gain=\frac{m0x+c0}{m1x+c1}\f$
 *
 * Where 'x' is the gain control parameter, and m0, m1, c0 and c1 are
 * image-sensor-specific constants of the sensor.
 * These constants are static parameters, and for any given image sensor either
 * m0 or m1 shall be zero.
 *
 * The full Gain equation therefore reduces to either:
 *
 * \f$gain=\frac{c0}{m1x+c1}\f$ or \f$\frac{m0x+c0}{c1}\f$
 */

/**
 * \var CameraSensorHelper::AnalogueGainExponential
 * \brief Gain is expressed using an exponential model
 *
 * The relationship between the integer gain parameter and the resulting gain
 * multiplier is given by the following equation:
 *
 * \f$gain = a \cdot 2^{m \cdot x}\f$
 *
 * Where 'x' is the gain control parameter, and 'a' and 'm' are image
 * sensor-specific constants.
 *
 * This is a subset of the MIPI CCS exponential gain model with the linear
 * factor 'a' being a constant, but with the exponent being configurable
 * through the 'm' coefficient.
 *
 * When the gain is expressed in dB, 'a' is equal to 1 and 'm' to
 * \f$log_{2}{10^{\frac{1}{20}}}\f$.
 */

/**
 * \struct CameraSensorHelper::AnalogueGainLinearConstants
 * \brief Analogue gain constants for the linear gain model
 *
 * \var CameraSensorHelper::AnalogueGainLinearConstants::m0
 * \brief Constant used in the linear gain coding/decoding
 *
 * \note Either m0 or m1 shall be zero.
 *
 * \var CameraSensorHelper::AnalogueGainLinearConstants::c0
 * \brief Constant used in the linear gain coding/decoding
 *
 * \var CameraSensorHelper::AnalogueGainLinearConstants::m1
 * \brief Constant used in the linear gain coding/decoding
 *
 * \note Either m0 or m1 shall be zero.
 *
 * \var CameraSensorHelper::AnalogueGainLinearConstants::c1
 * \brief Constant used in the linear gain coding/decoding
 */

/**
 * \struct CameraSensorHelper::AnalogueGainExpConstants
 * \brief Analogue gain constants for the exponential gain model
 *
 * \var CameraSensorHelper::AnalogueGainExpConstants::a
 * \brief Constant used in the exponential gain coding/decoding
 *
 * \var CameraSensorHelper::AnalogueGainExpConstants::m
 * \brief Constant used in the exponential gain coding/decoding
 */

/**
 * \struct CameraSensorHelper::AnalogueGainConstants
 * \brief Analogue gain model constants
 *
 * This union stores the constants used to calculate the analogue gain. The
 * CameraSensorHelper::gainType_ variable selects which union member is valid.
 *
 * \var CameraSensorHelper::AnalogueGainConstants::linear
 * \brief Constants for the linear gain model
 *
 * \var CameraSensorHelper::AnalogueGainConstants::exp
 * \brief Constants for the exponential gain model
 */

/**
 * \var CameraSensorHelper::gainType_
 * \brief The analogue gain model type
 */

/**
 * \var CameraSensorHelper::gainConstants_
 * \brief The analogue gain parameters used for calculation
 *
 * The analogue gain is calculated through a formula, and its parameters are
 * sensor specific. Use this variable to store the values at init time.
 */

/**
 * \class CameraSensorHelperFactoryBase
 * \brief Base class for camera sensor helper factories
 *
 * The CameraSensorHelperFactoryBase class is the base of all specializations of
 * the CameraSensorHelperFactory class template. It implements the factory
 * registration, maintains a registry of factories, and provides access to the
 * registered factories.
 */

/**
 * \brief Construct a camera sensor helper factory base
 * \param[in] name Name of the camera sensor helper class
 *
 * Creating an instance of the factory base registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used to look up factories and shall be unique.
 */
CameraSensorHelperFactoryBase::CameraSensorHelperFactoryBase(const std::string name)
	: name_(name)
{
	registerType(this);
}

/**
 * \brief Create an instance of the CameraSensorHelper corresponding to
 * a named factory
 * \param[in] name Name of the factory
 *
 * \return A unique pointer to a new instance of the CameraSensorHelper subclass
 * corresponding to the named factory or a null pointer if no such factory
 * exists
 */
std::unique_ptr<CameraSensorHelper> CameraSensorHelperFactoryBase::create(const std::string &name)
{
	const std::vector<CameraSensorHelperFactoryBase *> &factories =
		CameraSensorHelperFactoryBase::factories();

	for (const CameraSensorHelperFactoryBase *factory : factories) {
		if (name != factory->name_)
			continue;

		return factory->createInstance();
	}

	return nullptr;
}

/**
 * \brief Add a camera sensor helper class to the registry
 * \param[in] factory Factory to use to construct the camera sensor helper
 *
 * The caller is responsible to guarantee the uniqueness of the camera sensor
 * helper name.
 */
void CameraSensorHelperFactoryBase::registerType(CameraSensorHelperFactoryBase *factory)
{
	std::vector<CameraSensorHelperFactoryBase *> &factories =
		CameraSensorHelperFactoryBase::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all camera sensor helper factories
 * \return The list of camera sensor helper factories
 */
std::vector<CameraSensorHelperFactoryBase *> &CameraSensorHelperFactoryBase::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on link
	 * order.
	 */
	static std::vector<CameraSensorHelperFactoryBase *> factories;
	return factories;
}

/**
 * \class CameraSensorHelperFactory
 * \brief Registration of CameraSensorHelperFactory classes and creation of instances
 * \tparam _Helper The camera sensor helper class type for this factory
 *
 * To facilitate discovery and instantiation of CameraSensorHelper classes, the
 * CameraSensorHelperFactory class implements auto-registration of camera sensor
 * helpers. Each CameraSensorHelper subclass shall register itself using the
 * REGISTER_CAMERA_SENSOR_HELPER() macro, which will create a corresponding
 * instance of a CameraSensorHelperFactory subclass and register it with the
 * static list of factories.
 */

/**
 * \fn CameraSensorHelperFactory::CameraSensorHelperFactory(const char *name)
 * \brief Construct a camera sensor helper factory
 * \param[in] name Name of the camera sensor helper class
 *
 * Creating an instance of the factory registers it with the global list of
 * factories, accessible through the CameraSensorHelperFactoryBase::factories()
 * function.
 *
 * The factory \a name is used to look up factories and shall be unique.
 */

/**
 * \fn CameraSensorHelperFactory::createInstance() const
 * \brief Create an instance of the CameraSensorHelper corresponding to the
 * factory
 *
 * \return A unique pointer to a newly constructed instance of the
 * CameraSensorHelper subclass corresponding to the factory
 */

/**
 * \def REGISTER_CAMERA_SENSOR_HELPER
 * \brief Register a camera sensor helper with the camera sensor helper factory
 * \param[in] name Sensor model name used to register the class
 * \param[in] helper Class name of CameraSensorHelper derived class to register
 *
 * Register a CameraSensorHelper subclass with the factory and make it available
 * to try and match sensors.
 */

/* -----------------------------------------------------------------------------
 * Sensor-specific subclasses
 */

#ifndef __DOXYGEN__

/*
 * Helper function to compute the m parameter of the exponential gain model
 * when the gain code is expressed in dB.
 */
static constexpr double expGainDb(double step)
{
	constexpr double log2_10 = 3.321928094887362;

	/*
	 * The gain code is expressed in step * dB (e.g. in 0.1 dB steps):
	 *
	 * G_code = G_dB/step = 20/step*log10(G_linear)
	 *
	 * Inverting the formula, we get
	 *
	 * G_linear = 10^(step/20*G_code) = 2^(log2(10)*step/20*G_code)
	 */
	return log2_10 * step / 20;
}

class CameraSensorHelperImx219 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx219()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 0, 256, -1, 256 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx219", CameraSensorHelperImx219)

class CameraSensorHelperImx258 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx258()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 0, 512, -1, 512 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx258", CameraSensorHelperImx258)

class CameraSensorHelperImx290 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx290()
	{
		gainType_ = AnalogueGainExponential;
		gainConstants_.exp = { 1.0, expGainDb(0.3) };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx290", CameraSensorHelperImx290)

class CameraSensorHelperImx296 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx296()
	{
		gainType_ = AnalogueGainExponential;
		gainConstants_.exp = { 1.0, expGainDb(0.1) };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx296", CameraSensorHelperImx296)

class CameraSensorHelperImx477 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx477()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 0, 1024, -1, 1024 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx477", CameraSensorHelperImx477)

class CameraSensorHelperOv2740 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv2740()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov2740", CameraSensorHelperOv2740)

class CameraSensorHelperOv5640 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5640()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 16 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5640", CameraSensorHelperOv5640)

class CameraSensorHelperOv5670 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5670()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5670", CameraSensorHelperOv5670)

class CameraSensorHelperOv5675 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5675()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5675", CameraSensorHelperOv5675)

class CameraSensorHelperOv5693 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5693()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 16 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5693", CameraSensorHelperOv5693)

class CameraSensorHelperOv8865 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv8865()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov8865", CameraSensorHelperOv8865)

class CameraSensorHelperOv13858 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv13858()
	{
		gainType_ = AnalogueGainLinear;
		gainConstants_.linear = { 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov13858", CameraSensorHelperOv13858)

#endif /* __DOXYGEN__ */

} /* namespace ipa */

} /* namespace libcamera */
