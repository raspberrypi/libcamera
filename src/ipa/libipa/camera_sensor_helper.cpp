/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google Inc.
 *
 * Helper class that performs sensor-specific
 * parameter computations
 */
#include "camera_sensor_helper.h"

#include <cmath>
#include <limits>

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
 * \fn CameraSensorHelper::CameraSensorHelper()
 * \brief Construct a CameraSensorHelper instance
 *
 * CameraSensorHelper derived class instances shall never be constructed
 * manually but always through the CameraSensorHelperFactoryBase::create()
 * function.
 */

/**
 * \fn CameraSensorHelper::blackLevel()
 * \brief Fetch the black level of the sensor
 *
 * This function returns the black level of the sensor scaled to a 16bit pixel
 * width. If it is unknown an empty optional is returned.
 *
 * \todo Fill the blanks and add pedestal values for all supported sensors. Once
 * done, drop the std::optional<>.
 *
 * Black levels are typically the result of the following phenomena:
 * - Pedestal added by the sensor to pixel values. They are typically fixed,
 *   sometimes programmable and should be reported in datasheets (but
 *   documentation is not always available).
 * - Dark currents and other physical effects that add charge to pixels in the
 *   absence of light. Those can depend on the integration time and the sensor
 *   die temperature, and their contribution to pixel values depend on the
 *   sensor gains.
 *
 * The pedestal is usually the value with the biggest contribution to the
 * overall black level. In most cases it is either known before or in rare cases
 * (there is not a single driver with such a control in the linux kernel) can be
 * queried from the sensor. This function provides that fixed, known value.
 *
 * \return The black level of the sensor, or std::nullopt if not known
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
	if (auto *l = std::get_if<AnalogueGainLinear>(&gain_)) {
		ASSERT(l->m0 == 0 || l->m1 == 0);

		return (l->c0 - l->c1 * gain) /
		       (l->m1 * gain - l->m0);
	} else if (auto *e = std::get_if<AnalogueGainExp>(&gain_)) {
		ASSERT(e->a != 0 && e->m != 0);

		return std::log2(gain / e->a) / e->m;
	} else {
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
	double gain = static_cast<double>(gainCode);

	if (auto *l = std::get_if<AnalogueGainLinear>(&gain_)) {
		ASSERT(l->m0 == 0 || l->m1 == 0);

		return (l->m0 * gain + l->c0) /
		       (l->m1 * gain + l->c1);
	} else if (auto *e = std::get_if<AnalogueGainExp>(&gain_)) {
		ASSERT(e->a != 0 && e->m != 0);

		return e->a * std::exp2(e->m * gain);
	} else {
		ASSERT(false);
		return 0.0;
	}
}

/**
 * \struct CameraSensorHelper::AnalogueGainLinear
 * \brief Analogue gain constants for the linear gain model
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
 *
 * \var CameraSensorHelper::AnalogueGainLinear::m0
 * \brief Constant used in the linear gain coding/decoding
 *
 * \note Either m0 or m1 shall be zero.
 *
 * \var CameraSensorHelper::AnalogueGainLinear::c0
 * \brief Constant used in the linear gain coding/decoding
 *
 * \var CameraSensorHelper::AnalogueGainLinear::m1
 * \brief Constant used in the linear gain coding/decoding
 *
 * \note Either m0 or m1 shall be zero.
 *
 * \var CameraSensorHelper::AnalogueGainLinear::c1
 * \brief Constant used in the linear gain coding/decoding
 */

/**
 * \struct CameraSensorHelper::AnalogueGainExp
 * \brief Analogue gain constants for the exponential gain model
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
 *
 * \var CameraSensorHelper::AnalogueGainExp::a
 * \brief Constant used in the exponential gain coding/decoding
 *
 * \var CameraSensorHelper::AnalogueGainExp::m
 * \brief Constant used in the exponential gain coding/decoding
 */

/**
 * \var CameraSensorHelper::blackLevel_
 * \brief The black level of the sensor
 * \sa CameraSensorHelper::blackLevel()
 */

/**
 * \var CameraSensorHelper::gain_
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

class CameraSensorHelperAr0144 : public CameraSensorHelper
{
public:
	CameraSensorHelperAr0144()
	{
		/* Power-on default value: 168 at 12bits. */
		blackLevel_ = 2688;
	}

	uint32_t gainCode(double gain) const override
	{
		/* The recommended minimum gain is 1.6842 to avoid artifacts. */
		gain = std::clamp(gain, 1.0 / (1.0 - 13.0 / 32.0), 18.45);

		/*
		 * The analogue gain is made of a coarse exponential gain in
		 * the range [2^0, 2^4] and a fine inversely linear gain in the
		 * range [1.0, 2.0[. There is an additional fixed 1.153125
		 * multiplier when the coarse gain reaches 2^2.
		 */

		if (gain > 4.0)
			gain /= 1.153125;

		unsigned int coarse = std::log2(gain);
		unsigned int fine = (1 - (1 << coarse) / gain) * 32;

		/* The fine gain rounding depends on the coarse gain. */
		if (coarse == 1 || coarse == 3)
			fine &= ~1;
		else if (coarse == 4)
			fine &= ~3;

		return (coarse << 4) | (fine & 0xf);
	}

	double gain(uint32_t gainCode) const override
	{
		unsigned int coarse = gainCode >> 4;
		unsigned int fine = gainCode & 0xf;
		unsigned int d1;
		double d2, m;

		switch (coarse) {
		default:
		case 0:
			d1 = 1;
			d2 = 32.0;
			m = 1.0;
			break;
		case 1:
			d1 = 2;
			d2 = 16.0;
			m = 1.0;
			break;
		case 2:
			d1 = 1;
			d2 = 32.0;
			m = 1.153125;
			break;
		case 3:
			d1 = 2;
			d2 = 16.0;
			m = 1.153125;
			break;
		case 4:
			d1 = 4;
			d2 = 8.0;
			m = 1.153125;
			break;
		}

		/*
		 * With infinite precision, the calculated gain would be exact,
		 * and the reverse conversion with gainCode() would produce the
		 * same gain code. In the real world, rounding errors may cause
		 * the calculated gain to be lower by an amount negligible for
		 * all purposes, except for the reverse conversion. Converting
		 * the gain to a gain code could then return the quantized value
		 * just lower than the original gain code. To avoid this, tests
		 * showed that adding the machine epsilon to the multiplier m is
		 * sufficient.
		 */
		m += std::numeric_limits<decltype(m)>::epsilon();

		return m * (1 << coarse) / (1.0 - (fine / d1) / d2);
	}

private:
	static constexpr double kStep_ = 16;
};
REGISTER_CAMERA_SENSOR_HELPER("ar0144", CameraSensorHelperAr0144)

class CameraSensorHelperAr0521 : public CameraSensorHelper
{
public:
	uint32_t gainCode(double gain) const override
	{
		gain = std::clamp(gain, 1.0, 15.5);
		unsigned int coarse = std::log2(gain);
		unsigned int fine = (gain / (1 << coarse) - 1) * kStep_;

		return (coarse << 4) | (fine & 0xf);
	}

	double gain(uint32_t gainCode) const override
	{
		unsigned int coarse = gainCode >> 4;
		unsigned int fine = gainCode & 0xf;

		return (1 << coarse) * (1 + fine / kStep_);
	}

private:
	static constexpr double kStep_ = 16;
};
REGISTER_CAMERA_SENSOR_HELPER("ar0521", CameraSensorHelperAr0521)

class CameraSensorHelperGc05a2 : public CameraSensorHelper
{
public:
	CameraSensorHelperGc05a2()
	{
		/* From datasheet: 64 at 10bits. */
		blackLevel_ = 4096;
		gain_ = AnalogueGainLinear{ 100, 0, 0, 1024 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("gc05a2", CameraSensorHelperGc05a2)

class CameraSensorHelperGc08a3 : public CameraSensorHelper
{
public:
	CameraSensorHelperGc08a3()
	{
		/* From datasheet: 64 at 10bits. */
		blackLevel_ = 4096;
		gain_ = AnalogueGainLinear{ 100, 0, 0, 1024 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("gc08a3", CameraSensorHelperGc08a3)

class CameraSensorHelperImx214 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx214()
	{
		/* From datasheet: 64 at 10bits. */
		blackLevel_ = 4096;
		gain_ = AnalogueGainLinear{ 0, 512, -1, 512 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx214", CameraSensorHelperImx214)

class CameraSensorHelperImx219 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx219()
	{
		/* From datasheet: 64 at 10bits. */
		blackLevel_ = 4096;
		gain_ = AnalogueGainLinear{ 0, 256, -1, 256 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx219", CameraSensorHelperImx219)

class CameraSensorHelperImx258 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx258()
	{
		/* From datasheet: 0x40 at 10bits. */
		blackLevel_ = 4096;
		gain_ = AnalogueGainLinear{ 0, 512, -1, 512 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx258", CameraSensorHelperImx258)

class CameraSensorHelperImx283 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx283()
	{
		/* From datasheet: 0x32 at 10bits. */
		blackLevel_ = 3200;
		gain_ = AnalogueGainLinear{ 0, 2048, -1, 2048 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx283", CameraSensorHelperImx283)

class CameraSensorHelperImx290 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx290()
	{
		/* From datasheet: 0xf0 at 12bits. */
		blackLevel_ = 3840;
		gain_ = AnalogueGainExp{ 1.0, expGainDb(0.3) };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx290", CameraSensorHelperImx290)

class CameraSensorHelperImx296 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx296()
	{
		gain_ = AnalogueGainExp{ 1.0, expGainDb(0.1) };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx296", CameraSensorHelperImx296)

class CameraSensorHelperImx327 : public CameraSensorHelperImx290
{
};
REGISTER_CAMERA_SENSOR_HELPER("imx327", CameraSensorHelperImx327)

class CameraSensorHelperImx335 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx335()
	{
		/* From datasheet: 0x32 at 10bits. */
		blackLevel_ = 3200;
		gain_ = AnalogueGainExp{ 1.0, expGainDb(0.3) };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx335", CameraSensorHelperImx335)

class CameraSensorHelperImx415 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx415()
	{
		gain_ = AnalogueGainExp{ 1.0, expGainDb(0.3) };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx415", CameraSensorHelperImx415)

class CameraSensorHelperImx462 : public CameraSensorHelperImx290
{
};
REGISTER_CAMERA_SENSOR_HELPER("imx462", CameraSensorHelperImx462)

class CameraSensorHelperImx477 : public CameraSensorHelper
{
public:
	CameraSensorHelperImx477()
	{
		gain_ = AnalogueGainLinear{ 0, 1024, -1, 1024 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("imx477", CameraSensorHelperImx477)

class CameraSensorHelperOv2685 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv2685()
	{
		/*
		 * The Sensor Manual doesn't appear to document the gain model.
		 * This has been validated with some empirical testing only.
		 */
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov2685", CameraSensorHelperOv2685)

class CameraSensorHelperOv2740 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv2740()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov2740", CameraSensorHelperOv2740)

class CameraSensorHelperOv4689 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv4689()
	{
		/* From datasheet: 0x40 at 12bits. */
		blackLevel_ = 1024;
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov4689", CameraSensorHelperOv4689)

class CameraSensorHelperOv5640 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5640()
	{
		/* From datasheet: 0x10 at 10bits. */
		blackLevel_ = 1024;
		gain_ = AnalogueGainLinear{ 1, 0, 0, 16 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5640", CameraSensorHelperOv5640)

class CameraSensorHelperOv5647 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5647()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 16 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5647", CameraSensorHelperOv5647)

class CameraSensorHelperOv5670 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5670()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5670", CameraSensorHelperOv5670)

class CameraSensorHelperOv5675 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5675()
	{
		/* From Linux kernel driver: 0x40 at 10bits. */
		blackLevel_ = 4096;
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5675", CameraSensorHelperOv5675)

class CameraSensorHelperOv5693 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv5693()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 16 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov5693", CameraSensorHelperOv5693)

class CameraSensorHelperOv64a40 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv64a40()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov64a40", CameraSensorHelperOv64a40)

class CameraSensorHelperOv8858 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv8858()
	{
		/*
		 * \todo Validate the selected 1/128 step value as it differs
		 * from what the sensor manual describes.
		 *
		 * See: https://patchwork.linuxtv.org/project/linux-media/patch/20221106171129.166892-2-nicholas@rothemail.net/#142267
		 */
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov8858", CameraSensorHelperOv8858)

class CameraSensorHelperOv8865 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv8865()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov8865", CameraSensorHelperOv8865)

class CameraSensorHelperOv13858 : public CameraSensorHelper
{
public:
	CameraSensorHelperOv13858()
	{
		gain_ = AnalogueGainLinear{ 1, 0, 0, 128 };
	}
};
REGISTER_CAMERA_SENSOR_HELPER("ov13858", CameraSensorHelperOv13858)

#endif /* __DOXYGEN__ */

} /* namespace ipa */

} /* namespace libcamera */
