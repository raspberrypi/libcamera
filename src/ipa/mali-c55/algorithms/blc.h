/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Ideas On Board
 *
 * Mali-C55 sensor offset (black level) correction
 */

#include "algorithm.h"

namespace libcamera {

namespace ipa::mali_c55::algorithms {

class BlackLevelCorrection : public Algorithm
{
public:
	BlackLevelCorrection();
	~BlackLevelCorrection() = default;

	int init(IPAContext &context, const YamlObject &tuningData) override;
	int configure(IPAContext &context,
		      const IPACameraSensorInfo &configInfo) override;
	void prepare(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     mali_c55_params_buffer *params) override;
	void process(IPAContext &context, const uint32_t frame,
		     IPAFrameContext &frameContext,
		     const mali_c55_stats_buffer *stats,
		     ControlList &metadata) override;

private:
	static constexpr uint32_t kMaxOffset = 0xfffff;

	bool tuningParameters_;
	uint32_t offset00;
	uint32_t offset01;
	uint32_t offset10;
	uint32_t offset11;
};

} /* namespace ipa::mali_c55::algorithms */
} /* namespace libcamera */
