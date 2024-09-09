/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * imx500_tensor_parser.h - Parser for imx500 tensors
 */

#include <memory>
#include <stdint.h>
#include <string>
#include <unordered_map>
#include <vector>

#include <libcamera/base/span.h>

namespace RPiController {

enum TensorType {
	InputTensor = 0,
	OutputTensor,
	Kpi,
};

struct Dimensions {
	uint8_t ordinal;
	uint16_t size;
	uint8_t serializationIndex;
	uint8_t padding;
};

struct IMX500OutputTensorInfo {
	uint32_t totalSize;
	uint32_t numTensors;
	std::string networkName;
	std::shared_ptr<float[]> data;
	std::vector<uint32_t> tensorDataNum;
	std::vector<std::vector<Dimensions>> vecDim;
	std::vector<uint32_t> numDimensions;
};

struct IMX500InputTensorInfo {
	unsigned int width;
	unsigned int height;
	unsigned int widthStride;
	unsigned int heightStride;
	unsigned int channels;
	unsigned int size;
	std::string networkName;
	std::shared_ptr<uint8_t[]> data;
};

struct IMX500Tensors {
	bool valid;
	unsigned int offset;
};

int imx500ParseOutputTensor(IMX500OutputTensorInfo &outputTensorInfo,
			    libcamera::Span<const uint8_t> outputTensor);
int imx500ParseInputTensor(IMX500InputTensorInfo &inputTensorInfo,
			   libcamera::Span<const uint8_t> inputTensor);
std::unordered_map<TensorType, IMX500Tensors> imx500SplitTensors(libcamera::Span<const uint8_t> tensors);

} /* namespace RPiController */
