/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * libIPA Gamma correction algorithm
 */

#include "gamma.h"

#include <numeric>

#include <libcamera/controls.h>

#include "libcamera/internal/value_node.h"

/**
 * \file gamma.h
 * \brief libipa implementation of a gamma curve correction algorithm
 */

namespace libcamera {

namespace ipa {

LOG_DEFINE_CATEGORY(Gamma)

namespace gamma {

/**
 * \struct ActiveState
 * \brief Active gamma correction algorithm state
 *
 * \var ActiveState::gamma
 * \brief The gamma correction value
 */

/**
 * \struct FrameContext
 * \brief Per-frame gamma correction settings
 *
 * \var FrameContext::gamma
 * \brief The gamma correction value applied for this frame
 *
 * \var FrameContext::update
 * \brief A flag instructing the algorithm to push an update to the hardware
 */

} /* namespace gamma */

/**
 * \brief The default gamma correction value
 */
const float kDefaultGamma = 2.2f;

/**
 * \class GammaAlgorithmBase
 * \brief Base class for GammaAlgorithm to implement non-templated functions
 *
 * This base class for GammeaAlgorithm allows us to implement non templated
 * functions. IPA specific implementations shall derive from GammaAlgorithm and
 * not this class.
 */

/**
 * \fn GammaAlgorithmBase::GammaAlgorithmBase
 * \brief Construct an instance of the class
 * \param[in] nLutNodes Set the number of function knee-points expected by the
 * IPA algorithm
 */

/**
 * \brief Initialise the algorithm with the given tuning data
 * \param[out] controls The ControlList into which this algorithm's supported
 * controls will be emplaced.
 * \param[in] tuningData The tuning data to use with the algorithm
 * \param[in] segments A vector of segment spacings to define a custom
 * X coordinate system for the curve
 *
 * Parse \a tuningData and \a segments to initialize the gamma correction curve.
 * The tuning data may contain a default gamma value to use; otherwise the value
 * of \a kDefaultGamma will be taken as the default. The piecewise linear
 * function will be applied on a number of knots whose position is described by
 * the optional \a segments argument, which describes each segment's relative
 * length.
 *
 * For example, if the gamma correction has to be applied on 16 equally spaced
 * sampling points, a \a segments array like:
 *
 * [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
 *
 * would result in evenly spaced knee-points along the X-axis.
 *
 * Hardware may expect the knee-points to be spaced more densely towards the
 * start of the curve and more sparsely towards the end, in which case an
 * alternative array might be:
 *
 * [1, 1, 1, 1, 2, 2, 2, 2, 4, 4, 4, 8, 8, 8, 8, 8]
 *
 * As the values in \a segments represent the distance between two knee-points
 * relative to the total distance between the first and last point, the length
 * of \a segments should be equal to the number of knee-points minus one.
 *;
 * If an IPA implementation doesnt't provide \a segments, the GammaAlgorithm
 * class consturcts an evenly-spaced default.
 *
 * IPA modules are expected to call this function as part of their
 * implementation of Algorithm::init()
 *
 * @return 0 on success, a negative error code otherwise
 */
int GammaAlgorithmBase::init(ControlInfoMap::Map &controls, const ValueNode &tuningData,
			     std::span<unsigned int> segments)
{
	/*
	 * If the caller doesn't pass in a segment list we simply construct the
	 * position of the knee-points assuming equally spaced segments.
	 */
	if (segments.empty()) {
		for (unsigned int i = 0; i < nLutNodes_; i++)
			kneePoints_[i] = static_cast<float>(i) / (nLutNodes_ - 1);
	} else {
		/*
		 * As segments holds the distance between the knee-points, we
		 * expect one fewer segment entries than we have LUT nodes.
		 */
		if (segments.size() != nLutNodes_ - 1)
			return -EINVAL;

		float total = std::accumulate(segments.begin(), segments.end(), 0.0f);
		float x = 0.0f;

		for (unsigned int i = 0; i < nLutNodes_; i++) {
			kneePoints_[i] = x / total;

			if (i < segments.size())
				x += segments[i];
		}
	}

	defaultGamma_ = tuningData["gamma"].get<float>(kDefaultGamma);
	controls[&controls::Gamma] = ControlInfo(0.1f, 10.0f, defaultGamma_);

	return 0;
}

/**
 * \brief Configure the gamma correction algorithm
 * \param[out] state The gamma correction algorithm's active state
 *
 * Reset to the default gamma correction value.
 *
 * IPA modules are expected to call this function as part of their
 * implementation of Algorithm::configure()
 */
void GammaAlgorithmBase::configure(gamma::ActiveState &state)
{
	state.gamma = defaultGamma_;
}

/**
 * \brief Queue a request to the gamma correction algorithm
 * \param[in] state The algorithm's active state
 * \param[in] frame The current frame number
 * \param[in] context The algorithm's frame context
 * \param[in] controls The ControlList that was queued with the request
 *
 * Queue a new request to the gamma correction algorithm and handle any relevant
 * controls that were queued. The only control currently handled is:
 *
 * - controls::Gamma
 *
 * If a control with that ID is queued the value is stored in \a state and
 * \a context.
 *
 * IPA modules are expected to call this function as part of their
 * implementation of Algorithm::queueRequest()
 */
void GammaAlgorithmBase::queueRequest(gamma::ActiveState &state,
				      const uint32_t frame,
				      gamma::FrameContext &context,
				      const ControlList &controls)
{
	if (frame == 0)
		context.update = true;

	const auto &gamma = controls.get(controls::Gamma);
	if (gamma) {
		state.gamma = *gamma;
		context.update = true;
		LOG(Gamma, Info) << "Set gamma to " << *gamma;
	}

	context.gamma = state.gamma;
}

/**
 * \brief Populate metadata with the gamma correction values for a frame
 * \param[in] context The frame context
 * \param[out] metadata The ControlList of metadata for a frame
 *
 * Report the gamma value used to calculate the correction curve that was
 * applied to a frame.
 */
void GammaAlgorithmBase::process(gamma::FrameContext &context, ControlList &metadata)
{
	metadata.set(controls::Gamma, context.gamma);
}

/**
 * \var GammaAlgorithmBase::nLutNodes_
 * \brief The number of gamma LUT sampling points
 */

/**
 * \var GammaAlgorithmBase::defaultGamma_
 * \brief The default gamma parameter
 */

/**
 * \var GammaAlgorithmBase::kneePoints_
 * \brief a vector holding the X-position of the knee points of the curve
 */

/**
 * \class GammaAlgorithm
 * \brief The libipa gamma correction algorithm
 * \tparam NLutNodes The number of gamma LUT sampling points
 * \tparam UQ The fixedpoint representation of the gamma correction values
 *
 * Gamma correction adjusts for the differences in the way light is perceived
 * by a camera and the human eye by applying a function to the input values.
 * The GammaAlgorithm class facilitates this by building a piecewise linear
 * function from a gamma parameter and supplying it in the hardware-specific
 * formats defined by the IPA algorithms.
 *
 * IPA modules are expected to store an instance of GammaAlgorithm as a class
 * member, templated with the format and number of knee-points in the PWL
 * expected by their hardware and then call its functions in their overload of
 * the Algorithm class's function.
 *
 * When an application queues a new value for the gamma parameter with a
 * Request, the GammaAlgorithm will recalculate and populate the new LUT to be
 * sent to the ISP.
 *
 * Useful links:
 * - https://www.cambridgeincolour.com/tutorials/gamma-correction.htm
 * - https://en.wikipedia.org/wiki/SRGB
 */

/**
 * \fn GammaAlgorithm::prepare()
 * \tparam T The type of data expected by the hardware's look-up table
 * \param[in] context The frame context
 * \param[out] lut The std::span into which to place the calculated look-up table
 */

} /* namespace ipa */

} /* namespace libcamera */
