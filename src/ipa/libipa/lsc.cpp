/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * libIPA Lsc algorithms
 */

#include "lsc.h"

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

#include "lsc_polynomial.h"
#include "lsc_table.h"

/**
 * \file lsc.h
 * \brief libipa LSC algorithm
 */

namespace libcamera {

LOG_DEFINE_CATEGORY(Lsc)

namespace ipa {

namespace lsc {

/**
 * \struct ActiveState
 * \brief The LSC active state
 *
 * \var ActiveState::enabled
 * \brief Boolean flag for the LscAlgorithm enable status
 */

/**
 * \struct FrameContext
 * \brief The LSC frame context
 *
 * \var FrameContext::enabled
 * \brief Boolean flag for the LscAlgorithm enable status
 *
 * \var FrameContext::update
 * \brief Boolean flag for the LscAlgorithm updated status
 */

/**
 * \typedef Components
 * \brief Associate colour components with a list of gains in register format
 * \tparam T The type used to store the gain values
 *
 * LSC tables are defined as a list of gain values associated to a colour
 * component.
 *
 * As different ISPs support different colour components (usually 'r', 'gr',
 * 'gb', 'b' or just 'r', 'g', 'b') this class associates a string
 * identifier for the colour component to a list of gains.
 *
 * Each key name shall match an entry in the tuning file.
 *
 * The list of keys is provided to the LscAlgorithm class using \a
 * LscDescriptor::keys.
 *
 * The gain values are expressed in fixed-point register format stored in
 * variabled of type \a T, as each platform potentially has a different register
 * width.
 */

/**
 * \typedef ComponentsMap
 * \brief Associate a colour temperature to a LSC table
 * \tparam T The type of the gain values
 *
 * An LSC table is generated during the tuning phase for a specific colour
 * temperature, and a tuning file usually contains LSC tables generated for
 * several different colour temperatures.
 */

} /* namespace lsc */

#ifndef __DOXYGEN__
template<typename T>
void interpolateVector(const std::vector<T> &a,
		       const std::vector<T> &b,
		       std::vector<T> &dest, double lambda)
{
	ASSERT(a.size() == b.size());
	dest.resize(a.size());
	for (size_t i = 0; i < a.size(); i++)
		dest[i] = a[i] * (1.0 - lambda) + b[i] * lambda;
}

template<>
void Interpolator<lsc::Components<uint16_t>>::
	interpolate(const lsc::Components<uint16_t> &a,
		    const lsc::Components<uint16_t> &b,
		    lsc::Components<uint16_t> &dest,
		    double lambda)
{
	for (auto const &[k, v] : a)
		interpolateVector(v, b.at(k), dest[k], lambda);
}

template<>
void Interpolator<lsc::Components<uint8_t>>::
	interpolate(const lsc::Components<uint8_t> &a,
		    const lsc::Components<uint8_t> &b,
		    lsc::Components<uint8_t> &dest,
		    double lambda)
{
	for (auto const &[k, v] : a)
		interpolateVector(v, b.at(k), dest[k], lambda);
}
#endif

/**
 * \class LscAlgorithmBase
 * \brief Base class for LscAlgorithm
 *
 * Base class for LscAlgorithm for non-templated functions implementation
 */

/**
 * \param[in] tuningData The tuning data
 * \param[in] controls The IPA list of supported controls
 * \param[in] descriptor The LSC engine descriptor
 *
 * Parse \a tuningData according to the settings specified in \a descriptor to
 * populate the LSC data and registers LSC controls in \a controls.
 *
 * \return 0 on success, a negative error code otherwise
 */
int LscAlgorithmBase::init(const ValueNode &tuningData, ControlInfoMap::Map &controls,
			   const LscDescriptor &descriptor)
{
	polynomial_ = false;

	std::string type = tuningData["type"].get<std::string>("table");
	if (type == "table") {
		impl_ = std::make_unique<LscTable>();
		LOG(Lsc, Debug) << "Using table-based Lsc";
	} else if (type == "polynomial") {
		impl_ = std::make_unique<LscPolynomial>();
		polynomial_ = true;
		LOG(Lsc, Debug) << "Using polynomial Lsc";
	} else {
		LOG(Lsc, Error) << "Unsupported Lsc algorithm '"
				<< type << "'";
		return -EINVAL;
	}

	const ValueNode &yamlSets = tuningData["sets"];
	if (!yamlSets.isList()) {
		LOG(Lsc, Error) << "'sets' parameter not found in tuning file";
		return -EINVAL;
	}

	int ret = impl_->parseLscData(yamlSets, descriptor);
	if (ret)
		return ret;

	controls[&controls::LensShadingCorrectionEnable] =
		ControlInfo(false, true, true);

	return 0;
}

/**
 * \brief Queue a request to the lsc algorithm
 * \param[in] state The lsc active state
 * \param[in] context The lsc frame context
 * \param[in] controls The list of controls associated with a Request
 *
 * Queue a new list of \a controls to the lsc algorithm.
 * The only supported control is controls::LensShadingCorrectionEnable.
 */
void LscAlgorithmBase::queueRequest(lsc::ActiveState &state,
				    lsc::FrameContext &context,
				    const ControlList &controls)
{
	const auto &lscEnable = controls.get(controls::LensShadingCorrectionEnable);
	if (lscEnable && *lscEnable != state.enabled) {
		state.enabled = *lscEnable;

		LOG(Lsc, Debug)
			<< (state.enabled ? "Enabling" : "Disabling") << " Lsc";

		context.update = true;
	}

	context.enabled = state.enabled;
}

/**
 * \brief Populate the list of lsc metadata
 * \param[in] context The lsc frame context
 * \param[in] metadata The list of metadata
 *
 * Populates the list of \a metadata with controls handled by the LscAlgorithm
 * class. The only supported metadata is controls::LensShadingCorrectionEnable.
 */
void LscAlgorithmBase::process(lsc::FrameContext &context, ControlList &metadata)
{
	metadata.set(controls::LensShadingCorrectionEnable, context.enabled);
}

/**
 * \var LscAlgorithmBase::impl_
 * \brief The LSC algorithm implementation
 *
 * There are two classes derived from LscImplementation, the LscTable and
 * LscPolynomial ones. Which one to instantiate is decided by parsing the tuning
 * file.
 */

/**
 * \var LscAlgorithmBase::polynomial_
 * \brief Boolean flag for polynomial LSC
 *
 * Set to true if polynomial LSC is in use.
 */

/**
 * \class LscAlgorithm
 * \brief libIPA LSC algorithm implementation
 * \tparam U The platform fixed-point register format representation
 *
 * Due to the optical characteristics of the lens, the light intensity received
 * by the sensor is not uniform. The Lens Shading Correction algorithm applies
 * multipliers to all pixels to compensate for the lens shading effect.
 *
 * The LscAlgorithm implements the libipa Lens Shading Correction algorithm
 * using an implementation of the LscImplementation interface.
 *
 * This class provides support for parsing the tuning file content and
 * generates tables indexed by colour temperature to store per-colour-channel
 * gains for the IPA algorithm to be able to program the LSC engine.
 *
 * The init() function parses the tuning file and loads the gain tables either
 * in tabular form (LscTable) or as radial polynomials (LscPolynomial). The gain
 * tables are indexed by colour temperature with per-colour-components vectors
 * of gain values or polynomial coefficients.
 *
 * The colour components names are IPA-implementation specific and depend on the
 * ISP LSC engine design. Some LSC engine support 4 colour components (r, gr,
 * gb, b), some only support 3 colour components (r, g, b). The name (and
 * number) of the expected colour components shall be provided to
 * LscAlgorithm::init() using the LscDescriptor::keys field.
 *
 * Example of a tabular lens shading tuning file with 'r', 'g' and 'b' colour
 * components. The gain table has been omitted here, but the expected number
 * of entries has to be equal to
 * LscDescriptor::numHSamples * LscDescriptor::numVSamples.
 *
 * \code{.yaml}
 * - Lsc:
 *    sets:
 *      - ct: 2500
 *        r: [
 *        	.. gains table omitted..
 *        ]
 *        g: [
 *        	.. gains table omitted..
 *        ]
 *        b: [
 *        	.. gains table omitted..
 *        ]
 *      - ct: 6500
 *        r: [
 *        	.. gains table omitted..
 *        ]
 *        g: [
 *        	.. gains table omitted..
 *        ]
 *        b: [
 *        	.. gains table omitted..
 *        ]
 * \endcode
 *
 * Example of a polynomial lens shading tuning file with 'r', 'gr', 'gb' and 'b'
 * colour components:
 *
 * \code{.yaml}
 * - Lsc:
 *    type: "polynomial"
 *    sets:
 *      - ct: 2500
 *        r:
 *          cx: 0.5006571711950275
 *          cy: 0.510093737499277
 *          k0: 1.5393282208428813
 *          k1: -1.1434559757908016
 *          k2: 4.332602305814554
 *          k3: 0.0
 *          k4: 0.0
 *        gr:
 *          cx: 0.5009320529087338
 *          cy: 0.511208038949085
 *          k0: 1.5634738574805407
 *          k1: -1.5623484259968348
 *          k2: 4.846686073656501
 *          k3: 0.0
 *          k4: 0.0
 *        gb:
 *          cx: 0.5012013290343839
 *          cy: 0.5128251541578288
 *          k0: 1.526147944919103
 *          k1: -1.4316976083689723
 *          k2: 4.792604063222728
 *          k3: 0.0
 *          k4: 0.0
 *        b:
 *          cx: 0.49864139511067784
 *          cy: 0.5162095081739346
 *          k0: 1.0405245474038738
 *          k1: 0.05618339879447103
 *          k2: 1.8792813594001752
 *          k3: 0.0
 *          k4: 0.0
 *      - ct: 6000
 *        r:
 *          cx: 0.5006202239353942
 *          cy: 0.5099531318307661
 *          k0: 1.4702946023945032
 *          k1: -0.8893767547927631
 *          k2: 3.920547732201387
 *          k3: 0.0
 *          k4: 0.0
 *        gr:
 *          cx: 0.500907874178317
 *          cy: 0.511084916024106
 *          k0: 1.5336172760559457
 *          k1: -1.39964026514435
 *          k2: 4.565487728954618
 *          k3: 0.0
 *          k4: 0.0
 *        gb:
 *          cx: 0.5011898608900477
 *          cy: 0.5126797906745105
 *          k0: 1.5013145790354843
 *          k1: -1.2747407173754124
 *          k2: 4.514682876897286
 *          k3: 0.0
 *          k4: 0.0
 *        b:
 *          cx: 0.4987561413116136
 *          cy: 0.5159619420778772
 *          k0: 1.0102986422191802
 *          k1: 0.13263449763985727
 *          k2: 1.686556107316064
 *          k3: 0.0
 *          k4: 0.0
 * \endcode
 *
 * The LSC polynomial definition is generated at tuning time in relation
 * to the full sensor pixel array. This needs to be specified in
 * LscDescriptor::sensorSize.
 *
 * At LscAlgorithm::configure() time the LSC tables are re-sampled on the
 * sensor's crop rectangle in use to adapt them to the configuration in use for
 * a streaming session. Polynomial LSC tables support re-sampling and can be
 * applied to any sensor configuration. Grid-based LSC tables cannot currently
 * be re-sampled and the configuration as parsed from the tuning file is used
 * for all sensor configurations providing best-effort results.
 *
 * \todo Implement grid based re-sampling
 *
 * After re-sampling, the LSC tables gain values are converted from their
 * floating point representation (LscImplementation::Components) to the
 * platform's register representation (lsc::Components<>). Grid-based LSC tables
 * currently already contain gains represented in register format, so no
 * quantization is necessary but only a simple cast is required.
 *
 * \todo Express gains in floating point format for grid-based LSC tables
 *
 * When the IPA algorithms wants to get access to the (re-sampled) tables to
 * program its LSC engine, it uses LscAlgorithm::interpolateComponents() to get
 * an LSC table interpolated by the LscAlgorithm class for the specified colour
 * temperature. If the algorithm wants to access the non-interpolated tables it
 * can retrieve them using LscAlgorithm::getComponents().
 */

/**
 * \fn LscAlgorithm::configure()
 * \brief Re-sample and quantize LSC data
 * \param[in] state The LSC active state
 * \param[in] analogCrop The current sensor analog crop rectangle
 * \param[in] xPos List of horizontal positions of the LSC grid nodes
 * \param[in] yPos List of vertical positions of the LSC grid nodes
 *
 * Re-sample the LSC data for an \a analogCrop and convert gains to their
 * register representation using the class template paramter \a U.
 *
 * LSC tables are generated at tuning time using a known sensor configuration.
 * When a new streaming session is started, it might use a different sensor
 * configuration for which the LSC tables need to be adjusted to.
 *
 * This function re-generates the LSC tables to adapt them to a new sensor
 * configuration, specifically it re-samples the LSC data for a new \a
 * analogCrop on a grid specified by \a xPos and \a yPos. Re-sampling of
 * LSC data is currently supported by polynomial-based LSC tables.
 *
 * \sa LscImplementation::sampleForCrop
 *
 * Once tables have been re-sampled, they get quantized to the platform's
 * fixed-point register representation using the LscAlgorithm template parameter
 * \a U.
 *
 * \return 0 on success, a negative error code otherwise
 */

/**
 * \fn LscAlgorithm::interpolateComponents
 * \brief Interpolate the LSC tables for a given colour temperature
 * \param[in] ct The colour temperature
 *
 * LSC tables are generated using different colour temperatures during the
 * tuning phase.
 *
 * This function returns the interpolated LSC data for a given \a ct
 * colour temperature.
 *
 * IPA algorithm can use this function to obtain a list of per-colour-component
 * gains to program their LSC engines with every time a significant enough
 * change in colour temperature is detected.
 *
 * Calling this function is only valid after LscAlgorithm::configure() has been
 * called. An empty components list is returned otherwise.
 *
 * \return The LSC gains table interpolated for temperature \a ct
 */

/**
 * \fn LscAlgorithm::getComponents
 * \brief Return the map of LSC data per-colour-temperature
 *
 * Calling this function is only valid after LscAlgorithm::configure() has been
 * called. An empty components list is returned otherwise.
 *
 * \return The map of LSC gains tables per-colour-temperature
 */

} /* namespace ipa */

} /* namespace libcamera */
