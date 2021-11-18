/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * camera_sensor_helper.cpp - Helper class that performs sensor-specific
 * parameter computations
 */
#include "camera_sensor_helper.h"

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
 * manually but always through the CameraSensorHelperFactory::create() function.
 */

/**
 * \brief Compute gain code from the analogue gain absolute value
 * \param[in] gain The real gain to pass
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations.
 *
 * The parameters come from the MIPI Alliance Camera Specification for
 * Camera Command Set (CCS).
 *
 * \return The gain code to pass to V4L2
 */
uint32_t CameraSensorHelper::gainCode(double gain) const
{
	ASSERT(analogueGainConstants_.m0 == 0 || analogueGainConstants_.m1 == 0);
	ASSERT(analogueGainConstants_.type == AnalogueGainLinear);

	return (analogueGainConstants_.c0 - analogueGainConstants_.c1 * gain) /
	       (analogueGainConstants_.m1 * gain - analogueGainConstants_.m0);
}

/**
 * \brief Compute the real gain from the V4L2 subdev control gain code
 * \param[in] gainCode The V4L2 subdev control gain
 *
 * This function aims to abstract the calculation of the gain letting the IPA
 * use the real gain for its estimations. It is the counterpart of the function
 * CameraSensorHelper::gainCode.
 *
 * The parameters come from the MIPI Alliance Camera Specification for
 * Camera Command Set (CCS).
 *
 * \return The real gain
 */
double CameraSensorHelper::gain(uint32_t gainCode) const
{
	ASSERT(analogueGainConstants_.m0 == 0 || analogueGainConstants_.m1 == 0);
	ASSERT(analogueGainConstants_.type == AnalogueGainLinear);

	return (analogueGainConstants_.m0 * static_cast<double>(gainCode) + analogueGainConstants_.c0) /
	       (analogueGainConstants_.m1 * static_cast<double>(gainCode) + analogueGainConstants_.c1);
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
 * \brief Gain is computed using exponential gain estimation
 * (introduced in CCS v1.1)
 *
 * Starting with CCS v1.1, Alternate Global Analogue Gain is also available.
 * If the image sensor supports it, then the global analogue gain can be
 * controlled by linear and exponential gain formula:
 *
 * \f$gain = analogLinearGainGlobal * 2^{analogExponentialGainGlobal}\f$
 * \todo not implemented in libipa
 */

/**
 * \struct CameraSensorHelper::AnalogueGainConstants
 * \brief Analogue gain constants used for gain calculation
 */

/**
 * \var CameraSensorHelper::AnalogueGainConstants::type
 * \brief Analogue gain calculation mode
 */

/**
 * \var CameraSensorHelper::AnalogueGainConstants::m0
 * \brief Constant used in the analogue Gain coding/decoding
 *
 * \note Either m0 or m1 shall be zero.
 */

/**
 * \var CameraSensorHelper::AnalogueGainConstants::c0
 * \brief Constant used in the analogue gain coding/decoding
 */

/**
 * \var CameraSensorHelper::AnalogueGainConstants::m1
 * \brief Constant used in the analogue gain coding/decoding
 *
 * \note Either m0 or m1 shall be zero.
 */

/**
 * \var CameraSensorHelper::AnalogueGainConstants::c1
 * \brief Constant used in the analogue gain coding/decoding
 */

/**
 * \var CameraSensorHelper::analogueGainConstants_
 * \brief The analogue gain parameters used for calculation
 *
 * The analogue gain is calculated through a formula, and its parameters are
 * sensor specific. Use this variable to store the values at init time.
 */

/**
 * \class CameraSensorHelperFactory
 * \brief Registration of CameraSensorHelperFactory classes and creation of instances
 *
 * To facilitate discovery and instantiation of CameraSensorHelper classes, the
 * CameraSensorHelperFactory class maintains a registry of camera sensor helper
 * sub-classes. Each CameraSensorHelper subclass shall register itself using the
 * REGISTER_CAMERA_SENSOR_HELPER() macro, which will create a corresponding
 * instance of a CameraSensorHelperFactory subclass and register it with the
 * static list of factories.
 */

/**
 * \brief Construct a camera sensor helper factory
 * \param[in] name Name of the camera sensor helper class
 *
 * Creating an instance of the factory registers it with the global list of
 * factories, accessible through the factories() function.
 *
 * The factory \a name is used for debug purpose and shall be unique.
 */
CameraSensorHelperFactory::CameraSensorHelperFactory(const std::string name)
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
std::unique_ptr<CameraSensorHelper> CameraSensorHelperFactory::create(const std::string &name)
{
	std::vector<CameraSensorHelperFactory *> &factories =
		CameraSensorHelperFactory::factories();

	for (CameraSensorHelperFactory *factory : factories) {
		if (name != factory->name_)
			continue;

		CameraSensorHelper *helper = factory->createInstance();
		return std::unique_ptr<CameraSensorHelper>(helper);
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
void CameraSensorHelperFactory::registerType(CameraSensorHelperFactory *factory)
{
	std::vector<CameraSensorHelperFactory *> &factories =
		CameraSensorHelperFactory::factories();

	factories.push_back(factory);
}

/**
 * \brief Retrieve the list of all camera sensor helper factories
 * \return The list of camera sensor helper factories
 */
std::vector<CameraSensorHelperFactory *> &CameraSensorHelperFactory::factories()
{
	/*
	 * The static factories map is defined inside the function to ensure
	 * it gets initialized on first use, without any dependency on link
	 * order.
	 */
	static std::vector<CameraSensorHelperFactory *> factories;
	return factories;
}

/**
 * \fn CameraSensorHelperFactory::createInstance()
 * \brief Create an instance of the CameraSensorHelper corresponding to the
 * factory
 *
 * This virtual function is implemented by the REGISTER_CAMERA_SENSOR_HELPER()
 * macro. It creates a camera sensor helper instance associated with the camera
 * sensor model.
 *
 * \return A pointer to a newly constructed instance of the CameraSensorHelper
 * subclass corresponding to the factory
 */

/**
 * \var CameraSensorHelperFactory::name_
 * \brief The name of the factory
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

class CameraSensorHelperImx219 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx219()
	{
		analogueGainConstants_ = { AnalogueGainLinear, 0, 256, -1, 256 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx219", CameraSensorHelperImx219)

class CameraSensorHelperImx258 : public CameraSensorHelper
{
public:
        CameraSensorHelperImx258()
        {
                analogueGainConstants_ = { AnalogueGainLinear, 0, 512, -1, 512 };
        }
};
REGISTER_CAMERA_SENSOR_HELPER("imx258", CameraSensorHelperImx258)

class CameraSensorHelperOv5670 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5670()
	{
		analogueGainConstants_ = { AnalogueGainLinear, 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5670", CameraSensorHelperOv5670)

class CameraSensorHelperOv5693 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5693()
	{
		analogueGainConstants_ = { AnalogueGainLinear, 1, 0, 0, 16 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5693", CameraSensorHelperOv5693)

class CameraSensorHelperOv8865 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv8865()
	{
		analogueGainConstants_ = { AnalogueGainLinear, 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov8865", CameraSensorHelperOv8865)

class CameraSensorHelperOv13858 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv13858()
	{
		analogueGainConstants_ = { AnalogueGainLinear, 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov13858", CameraSensorHelperOv13858)

#endif /* __DOXYGEN__ */

} /* namespace ipa */

} /* namespace libcamera */
