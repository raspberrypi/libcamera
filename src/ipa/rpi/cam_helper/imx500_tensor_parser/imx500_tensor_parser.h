/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_tensor_parser.h - Parser for imx500 tensors
 */

#include <stdint.h>
#include <unordered_map>
#include <vector>

#include <libcamera/base/span.h>

namespace RPiController {

/* Setup in the IMX500 driver */
constexpr unsigned int TensorStride = 4064;

enum TensorType {
	InputTensor = 0,
	OutputTensor
};

struct IMX500OutputTensorInfo {
	uint32_t totalSize;
	uint32_t numTensors;
	std::vector<float> address;
	std::vector<uint32_t> tensorDataNum;
};

int imx500ParseOutputTensor(IMX500OutputTensorInfo &outputTensorInfo,
			    libcamera::Span<const uint8_t> outputTensor);

std::unordered_map<unsigned int, unsigned int> imx500SplitTensors(libcamera::Span<const uint8_t> tensors);

} /* namespace RPiController */
