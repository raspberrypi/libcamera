/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Bryan O'Donoghue.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#pragma once

#include <memory>
#include <stdint.h>
#include <vector>

#define GL_GLEXT_PROTOTYPES
#define EGL_EGLEXT_PROTOTYPES
#include <libcamera/base/object.h>

#include "libcamera/internal/bayer_format.h"
#include "libcamera/internal/egl.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/software_isp/benchmark.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl32.h>

#include "debayer.h"

namespace libcamera {

#define DEBAYER_EGL_MIN_SIMPLE_RGB_GAIN_TEXTURE_UNITS 4
#define DEBAYER_OPENGL_COORDS 4

class DebayerEGL : public Debayer
{
public:
	DebayerEGL(std::unique_ptr<SwStatsCpu> stats, const GlobalConfiguration &configuration);
	~DebayerEGL();

	int configure(const StreamConfiguration &inputCfg,
		      const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
		      bool ccmEnabled);

	Size patternSize(PixelFormat inputFormat);

	std::vector<PixelFormat> formats(PixelFormat input);
	std::tuple<unsigned int, unsigned int> strideAndFrameSize(const PixelFormat &outputFormat, const Size &size);

	void process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params);
	int start();
	void stop();

	const SharedFD &getStatsFD() { return stats_->getStatsFD(); }
	unsigned int frameSize();

	SizeRange sizes(PixelFormat inputFormat, const Size &inputSize);

private:
	static int getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config);
	static int getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config);
	int setupStandardBayerOrder(BayerFormat::Order order);
	void pushEnv(std::vector<std::string> &shaderEnv, const char *str);
	int initBayerShaders(PixelFormat inputFormat, PixelFormat outputFormat);
	int initEGLContext();
	int generateTextures();
	int compileShaderProgram(GLuint &shaderId, GLenum shaderType,
				 unsigned char *shaderData, int shaderDataLen,
				 std::vector<std::string> shaderEnv);
	int linkShaderProgram(void);
	int getShaderVariableLocations();
	void setShaderVariableValues(DebayerParams &params);
	void configureTexture(GLuint &texture);
	int debayerGPU(MappedFrameBuffer &in, int out_fd, DebayerParams &params);

	/* Shader program identifiers */
	GLuint vertexShaderId_ = 0;
	GLuint fragmentShaderId_ = 0;
	GLuint programId_ = 0;

	/* Pointer to object representing input texture */
	std::unique_ptr<eGLImage> eglImageBayerIn_;
	std::unique_ptr<eGLImage> eglImageBayerOut_;

	/* Shader parameters */
	float firstRed_x_;
	float firstRed_y_;
	GLint attributeVertex_;
	GLint attributeTexture_;
	GLint textureUniformStep_;
	GLint textureUniformSize_;
	GLint textureUniformStrideFactor_;
	GLint textureUniformBayerFirstRed_;
	GLint textureUniformProjMatrix_;

	GLint textureUniformBayerDataIn_;

	/* Represent per-frame CCM as a uniform vector of floats 3 x 3 */
	GLint ccmUniformDataIn_;

	/* Black Level compensation */
	GLint blackLevelUniformDataIn_;

	/* Gamma */
	GLint gammaUniformDataIn_;

	/* Contrast */
	GLint contrastExpUniformDataIn_;

	Rectangle window_;
	std::unique_ptr<SwStatsCpu> stats_;
	eGL egl_;
	uint32_t width_;
	uint32_t height_;
	GLint glFormat_;
	unsigned int bytesPerPixel_;
	uint32_t shaderStridePixels_;
};

} /* namespace libcamera */
