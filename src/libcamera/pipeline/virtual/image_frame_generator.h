/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * Derived class of FrameGenerator for generating frames from images
 */

#pragma once

#include <filesystem>
#include <memory>
#include <stdint.h>
#include <sys/types.h>
#include <vector>

#include "frame_generator.h"

namespace libcamera {

/* Frame configuration provided by the config file */
struct ImageFrames {
	std::vector<std::filesystem::path> files;
};

class ImageFrameGenerator : public FrameGenerator
{
public:
	static std::unique_ptr<ImageFrameGenerator> create(ImageFrames &imageFrames);

private:
	static constexpr unsigned int frameRepeat = 4;

	struct ImageFrameData {
		std::unique_ptr<uint8_t[]> Y;
		std::unique_ptr<uint8_t[]> UV;
		Size size;
	};

	void configure(const Size &size) override;
	int generateFrame(const Size &size, const FrameBuffer *buffer) override;

	std::vector<ImageFrameData> imageFrameDatas_;
	std::vector<ImageFrameData> scaledFrameDatas_;
	ImageFrames *imageFrames_;
	unsigned int frameIndex_;
	unsigned int parameter_;
};

} /* namespace libcamera */
