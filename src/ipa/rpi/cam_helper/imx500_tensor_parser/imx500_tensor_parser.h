/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_tensor_parser.h - Parser for imx500 tensors
 */

#include <stdint.h>
#include <vector>

#include <libcamera/base/span.h>

namespace RPiController {

struct IMX500OutputTensorInfo {
	uint32_t totalSize;
	uint32_t numTensors;
	std::vector<float> address;
	std::vector<uint32_t> tensorDataNum;
};

int imx500ParseOutputTensor(IMX500OutputTensorInfo &outputTensorInfo,
			    libcamera::Span<const uint8_t> outputTensor);

} /* namespace RPiController */
