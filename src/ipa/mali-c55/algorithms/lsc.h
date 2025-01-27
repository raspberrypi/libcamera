/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board Oy
 *
 * lsc.h - Mali-C55 Lens shading correction algorithm
 */

#include <map>
#include <tuple>

#include "algorithm.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

class Lsc : public Algorithm
{
public:
	Lsc() = default;
	~Lsc() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     mali_c55_params_buffer *params) override;
private:
	static constexpr unsigned int kRedOffset = 0;
	static constexpr unsigned int kGreenOffset = 1024;
	static constexpr unsigned int kBlueOffset = 2048;

	size_t fillConfigParamsBlock(mali_c55_params_block block) const;
	size_t fillSelectionParamsBlock(mali_c55_params_block block,
					uint8_t bank, uint8_t alpha) const;
	std::tuple<uint8_t, uint8_t> findBankAndAlpha(uint32_t ct) const;

	std::vector<uint32_t> mesh_ = std::vector<uint32_t>(3072);
	std::vector<uint32_t> colourTemperatures_;
	uint32_t meshScale_;
	uint32_t meshSize_;
};

} /* namespace ipa::mali_c55::algorithms */

} /* namespace libcamera */
