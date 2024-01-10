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

enum TensorType {
	InputTensor = 0,
	OutputTensor
};

struct IMX500OutputTensorInfo {
	uint32_t totalSize;
	uint32_t numTensors;
	std::vector<float> data;
	std::vector<uint32_t> tensorDataNum;
};

struct IMX500InputTensorInfo {
	unsigned int width;
	unsigned int height;
	unsigned int widthStride;
	unsigned int heightStride;
	unsigned int channels;
	unsigned int size;
	std::vector<uint8_t> data;
};

int imx500ParseOutputTensor(IMX500OutputTensorInfo &outputTensorInfo,
			    libcamera::Span<const uint8_t> outputTensor);
int imx500ParseInputTensor(IMX500InputTensorInfo &inputTensorInfo,
			   libcamera::Span<const uint8_t> inputTensor);
std::unordered_map<unsigned int, unsigned int> imx500SplitTensors(libcamera::Span<const uint8_t> tensors);

} /* namespace RPiController */
