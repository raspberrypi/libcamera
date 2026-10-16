/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * libIPA CCM algorithm
 */

#include "ccm.h"

/**
 * \file ccm.h
 * \brief libipa CCM (Colour Correction Matrix) algorithm
 */

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(Ccm)

namespace ccm {

/**
 * \struct ActiveState
 * \brief Active CCM state
 *
 * \var ActiveState::manual
 * \brief The most recent manually requested CCM state
 *
 * \var ActiveState::automatic
 * \brief The most recent automatically calculated CCM state
 */

/**
 * \struct CcmContext
 * \brief CCM coefficients and offsets
 *
 * \var CcmContext::ccm
 * \brief Matrix of 3x3 CCM coefficients
 *
 * \var CcmContext::offsets
 * \brief Vector of RGB CCM offsets
 */

/**
 * \typedef FrameContext
 * \brief Per-frame CCM state
 */

} /* namespace ccm */

/**
 * \class CcmAlgorithmBase
 * \brief Base class for CcmAlgorithm for non-templated functions implementation
 *
 * Base class for CcmAlgorithm where non-templated functions are implemented.
 * IPA implementations shall use CcmAlgorithm and not this class.
 */

/**
 * \brief Initialize the algorithm with the given tuning data
 * \param[in] tuningData The tuning data to use for the algorithm
 *
 * Parse \a tuningData to initialize the CCM algorithm and register controls.
 * IPA modules are expected to call this function as part of their
 * implementation of Algorithm::init().
 *
 * \return 0 on success, a negative error code otherwise
 */
int CcmAlgorithmBase::init(const ValueNode &tuningData)
{
	int ret = ccm_.readYaml(tuningData["ccms"], "ct", "ccm");
	if (ret < 0) {
		LOG(Ccm, Warning)
			<< "Failed to parse 'ccm' "
			<< "parameter from tuning file; falling back to unit matrix";
		ccm_.setData({ { 0, Matrix<float, 3, 3>::identity() } });
	}

	ret = offsets_.readYaml(tuningData["ccms"], "ct", "offsets");
	if (ret < 0) {
		LOG(Ccm, Warning)
			<< "Failed to parse 'offsets' "
			<< "parameter from tuning file; falling back to zero offsets";

		offsets_.setData({ { 0, Matrix<int16_t, 3, 1>({ 0, 0, 0 }) } });
	}

	return 0;
}

/**
 * \brief Configure the CCM algorithm
 * \param[in] state The CCM active state
 * \param[in] temperatureK The colour temperature in Kelvin
 *
 * Configure the CCM algorithm by initializing the manual and automatic
 * states in \a state by interpolating the default colour correction matrix
 * with the given colour temperature \a temperatureK.
 *
 * \return 0 if successful, an error code otherwise
 */
int CcmAlgorithmBase::configure(ccm::ActiveState &state, unsigned int temperatureK)
{
	state.manual.ccm = ccm_.getInterpolated(temperatureK);
	state.manual.offsets = offsets_.getInterpolated(temperatureK);
	state.automatic.ccm = ccm_.getInterpolated(temperatureK);
	state.automatic.offsets = offsets_.getInterpolated(temperatureK);

	return 0;
}

/**
 * \brief Queue a Request to the CCM algorithm
 * \param[in] state The CCM active state
 * \param[in] context The CCM frame context
 * \param[in] controls The list of controls associated with the Request
 *
 * Queue a new Request to the CCM algorithm and store the manual colour
 * correction matrix and temperature in \a frameContext.
 *
 * The currently handled controls are:
 * - controls::ColourTemperature
 * - controls::ColourCorrectionMatrix
 *
 * When controls::ColourCorrectionMatrix is passed in the supplied matrix is
 * stored in \a state and \a context.
 *
 * When controls::ColourTemperature is passed in, the matrices loaded from
 * configuration file are interpolated with the given temperature and the result
 * is stored in \a state and \a context.
 *
 * If the IPA is running in manual mode, the IPA CCM algorithm implementations
 * can use the matrix coefficients and offsets directly from \a context after
 * calling this function to program the HW CCM engine, without calling prepare().
 */
void CcmAlgorithmBase::queueRequest(ccm::ActiveState &state,
				    ccm::FrameContext &context,
				    const ControlList &controls)
{
	const auto &colourTemperature = controls.get(controls::ColourTemperature);
	const auto &ccmMatrix = controls.get(controls::ColourCorrectionMatrix);
	if (ccmMatrix) {
		state.manual.ccm = Matrix<float, 3, 3>(*ccmMatrix);
		LOG(Ccm, Debug) << "Setting manual CCM from CCM control to "
				<< state.manual.ccm;
	} else if (colourTemperature) {
		state.manual.ccm = ccm_.getInterpolated(*colourTemperature);
		LOG(Ccm, Debug) << "Setting manual CCM from CT control to "
				<< state.manual.ccm;
	}

	context = state.manual;
}

/**
 * \brief Calculate the matrix coefficients for a colour temperature
 * \param[in] state The CCM active state
 * \param[in] context The CCM frame context
 * \param[in] frame The frame number
 * \param[in] temperatureK The colour temperature in Kelvin
 *
 * Interpolate the colour correction matrices as loaded from configuration file
 * for colour temperature \a temperatureK.
 *
 * The function shall only be called if the IPA algorithm is running in auto
 * mode. If running in manual mode the application supplied correction matrix is
 * stored in \a frameContext at queueRequest() time.
 */
void CcmAlgorithmBase::prepare(ccm::ActiveState &state,
			       ccm::FrameContext &context,
			       unsigned int frame, unsigned int temperatureK)
{
	if (frame > 0 && temperatureK == ct_) {
		context = state.automatic;
		return;
	}

	ct_ = temperatureK;
	context.ccm = ccm_.getInterpolated(ct_);
	context.offsets = offsets_.getInterpolated(ct_);

	state.automatic = context;
}

/**
 * \brief Populate metadata with the latest correction matrix coefficients
 * \param[in] context The CCM frame context
 * \param[out] metadata The metadata list
 */
void CcmAlgorithmBase::process(ccm::FrameContext &context, ControlList &metadata)
{
	metadata.set(controls::ColourCorrectionMatrix, context.ccm.data());
}

/**
 * \class CcmAlgorithm
 * \brief The libipa CCM algorithm
 * \tparam Q The fixedpoint register representation of the colour correction
 * coefficients
 *
 * Implement the CCM algorithm for libipa.
 *
 * The CcmAlgorithm class implements an interface similar in spirit to the one
 * of the Algorithm class. IPA modules are expected to store an instance of
 * CcmAlgorithm as class member, template it with the CCM coefficients register
 * representation and call its functions in their implementations of the
 * Algorithm interface.
 *
 * The CcmAlgorithm class provides an init() function where tuning data is
 * parsed and the per-colour temperature correction matrices are loaded from
 * the tuning file.
 *
 * CcmAlgorithm supports both automatic and manual colour correction operations,
 * but doesn't offer a way to select one of them. Enabling or disabling
 * automatic CCM operations usually goes through the Awb algorithm
 * enable/disable as the two algorithms should work with the same mode.
 *
 * When the IPA module runs in manual mode a custom colour correction matrix
 * or a custom colour temperature can be supplied to the CCM algorithm at
 * queueRequest() time. If the Request contains a color correction matrix
 * (controls::ColourCorrectionMatrix) then the matrix coefficients gets saved in
 * the FrameContext and the IPA module can immediately use them and doesn't need
 * to call prepare(). If a custom colour temperature is provided
 * (controls::ColourTemperature) then the matrices loaded from configuration are
 * interpolated with it and the result is saved in the FrameContext. In this
 * case as well IPA modules can use the result immediately and should avoid
 * calling prepare().
 *
 * When the IPA module runs in automatic mode instead, it estimates the scene
 * colour temperature. The estimated colour temperature shall be passed to
 * prepare(), where it is used to interpolate the matrices loaded from the
 * tuning file. The resulting coefficients are stored in the FrameContext for
 * the IPA algorithm to use them to program their CCM engine registers.
 */

/**
 * \fn CcmAlgorithm::init()
 * \param[in] controls The info map of the IPA controls
 * \copydoc CcmAlgorithmBase::init()
 */

} /* namespace ipa */

} /* namespace libcamera */
