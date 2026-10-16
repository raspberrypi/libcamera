/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2026 Ideas on Board Oy
 *
 * libIPA Lsc algorithm
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/controls.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/value_node.h"

#include "interpolator.h"
#include "lsc_base.h"

namespace libcamera {

namespace ipa {

namespace lsc {

struct ActiveState {
	bool enabled;
};

struct FrameContext {
	bool enabled;
	bool update;
};

template<typename T>
using Components = std::map<std::string, std::vector<T>, std::less<>>;

template<typename T>
using ComponentsMap = std::map<unsigned int, Components<T>>;

} /* namespace lsc */

#ifndef __DOXYGEN__
template<>
void Interpolator<lsc::Components<uint16_t>>::
	interpolate(const lsc::Components<uint16_t> &a,
		    const lsc::Components<uint16_t> &b,
		    lsc::Components<uint16_t> &dest,
		    double lambda);

template<>
void Interpolator<lsc::Components<uint8_t>>::
	interpolate(const lsc::Components<uint8_t> &a,
		    const lsc::Components<uint8_t> &b,
		    lsc::Components<uint8_t> &dest,
		    double lambda);
#endif /* __DOXYGEN__ */

class LscAlgorithmBase
{
public:
	int init(const ValueNode &tuningData, ControlInfoMap::Map &controls,
		 const LscDescriptor &descriptor);

	void queueRequest(lsc::ActiveState &state, lsc::FrameContext &context,
			  const ControlList &controls);
	void process(lsc::FrameContext &context, ControlList &metadata);

protected:
	LscAlgorithmBase() = default;

	std::unique_ptr<LscImplementation> impl_;
	bool polynomial_;
};

template<typename U>
class LscAlgorithm : public LscAlgorithmBase
{
private:
	using Components = lsc::Components<typename U::QuantizedType>;
	using ComponentsMap = lsc::ComponentsMap<typename U::QuantizedType>;

public:
	LscAlgorithm() = default;

	int configure(lsc::ActiveState &state, const Rectangle &analogCrop,
		      const std::vector<double> &xPos,
		      const std::vector<double> &yPos)
	{
		LscImplementation::ComponentsMap data =
			impl_->sampleForCrop(analogCrop, xPos, yPos);

		ComponentsMap lscData;
		for (const auto &[t, c] : data) {
			Components &comp = lscData[t];

			for (const auto &[k, gains] : c) {
				auto &quantizedGains = comp[k];
				quantizedGains.reserve(gains.size());

				for (const float &gain : gains) {
					/*
					 * Tabular LSC tables already store
					 * quantized values.
					 */
					if (polynomial_)
						quantizedGains.push_back(U(gain).quantized());
					else
						quantizedGains.push_back(gain);
				}
			}
		}

		sets_.setData(std::move(lscData));
		state.enabled = true;

		return 0;
	}

	const Components interpolateComponents(unsigned int ct)
	{
		return sets_.getInterpolated(ct);
	}

	const ComponentsMap &getComponents() const
	{
		return sets_.data();
	}

private:
	Interpolator<Components> sets_;
};

} /* namespace ipa */

} /* namespace libcamera */
