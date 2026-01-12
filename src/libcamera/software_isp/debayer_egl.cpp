/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#include "debayer_egl.h"

#include <cmath>
#include <stdlib.h>
#include <time.h>

#include <libcamera/base/utils.h>

#include <libcamera/formats.h>

#include "../glsl_shaders.h"

namespace libcamera {

/**
 * \class DebayerEGL
 * \brief Class for debayering using an EGL Shader
 *
 * Implements an EGL shader based debayering solution.
 */

/**
 * \fn DebayerEGL::DebayerEGL(std::unique_ptr<SwStatsCpu> stats, const GlobalConfiguration &configuration)
 * \brief Construct a DebayerEGL object
 * \param[in] stats Statistics processing object
 * \param[in] configuration Global configuration reference
 */
DebayerEGL::DebayerEGL(std::unique_ptr<SwStatsCpu> stats, const GlobalConfiguration &configuration)
	: Debayer(configuration), stats_(std::move(stats))
{
}

DebayerEGL::~DebayerEGL()
{
}

int DebayerEGL::getInputConfig(PixelFormat inputFormat, DebayerInputConfig &config)
{
	BayerFormat bayerFormat =
		BayerFormat::fromPixelFormat(inputFormat);

	if ((bayerFormat.bitDepth == 8 || bayerFormat.bitDepth == 10) &&
	    bayerFormat.packing == BayerFormat::Packing::None &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = (bayerFormat.bitDepth + 7) & ~7;
		config.patternSize.width = 2;
		config.patternSize.height = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::XRGB8888,
								  formats::ARGB8888,
								  formats::XBGR8888,
								  formats::ABGR8888 });
		return 0;
	}

	if (bayerFormat.bitDepth == 10 &&
	    bayerFormat.packing == BayerFormat::Packing::CSI2 &&
	    isStandardBayerOrder(bayerFormat.order)) {
		config.bpp = 10;
		config.patternSize.width = 4; /* 5 bytes per *4* pixels */
		config.patternSize.height = 2;
		config.outputFormats = std::vector<PixelFormat>({ formats::XRGB8888,
								  formats::ARGB8888,
								  formats::XBGR8888,
								  formats::ABGR8888 });
		return 0;
	}

	LOG(Debayer, Error)
		<< "Unsupported input format " << inputFormat;

	return -EINVAL;
}

int DebayerEGL::getOutputConfig(PixelFormat outputFormat, DebayerOutputConfig &config)
{
	if (outputFormat == formats::XRGB8888 || outputFormat == formats::ARGB8888 ||
	    outputFormat == formats::XBGR8888 || outputFormat == formats::ABGR8888) {
		config.bpp = 32;
		return 0;
	}

	LOG(Debayer, Error)
		<< "Unsupported output format " << outputFormat;

	return -EINVAL;
}

int DebayerEGL::getShaderVariableLocations(void)
{
	attributeVertex_ = glGetAttribLocation(programId_, "vertexIn");
	attributeTexture_ = glGetAttribLocation(programId_, "textureIn");

	textureUniformBayerDataIn_ = glGetUniformLocation(programId_, "tex_y");
	ccmUniformDataIn_ = glGetUniformLocation(programId_, "ccm");
	blackLevelUniformDataIn_ = glGetUniformLocation(programId_, "blacklevel");
	gammaUniformDataIn_ = glGetUniformLocation(programId_, "gamma");
	contrastExpUniformDataIn_ = glGetUniformLocation(programId_, "contrastExp");

	textureUniformStep_ = glGetUniformLocation(programId_, "tex_step");
	textureUniformSize_ = glGetUniformLocation(programId_, "tex_size");
	textureUniformStrideFactor_ = glGetUniformLocation(programId_, "stride_factor");
	textureUniformBayerFirstRed_ = glGetUniformLocation(programId_, "tex_bayer_first_red");
	textureUniformProjMatrix_ = glGetUniformLocation(programId_, "proj_matrix");

	LOG(Debayer, Debug) << "vertexIn " << attributeVertex_ << " textureIn " << attributeTexture_
			    << " tex_y " << textureUniformBayerDataIn_
			    << " ccm " << ccmUniformDataIn_
			    << " blacklevel " << blackLevelUniformDataIn_
			    << " gamma " << gammaUniformDataIn_
			    << " contrastExp " << contrastExpUniformDataIn_
			    << " tex_step " << textureUniformStep_
			    << " tex_size " << textureUniformSize_
			    << " stride_factor " << textureUniformStrideFactor_
			    << " tex_bayer_first_red " << textureUniformBayerFirstRed_
			    << " proj_matrix " << textureUniformProjMatrix_;
	return 0;
}

int DebayerEGL::initBayerShaders(PixelFormat inputFormat, PixelFormat outputFormat)
{
	std::vector<std::string> shaderEnv;
	unsigned int fragmentShaderDataLen = 0;
	const unsigned char *fragmentShaderData = 0;
	unsigned int vertexShaderDataLen = 0;
	const unsigned char *vertexShaderData = 0;
	GLenum err;

	/* Target gles 100 glsl requires "#version x" as first directive in shader */
	egl_.pushEnv(shaderEnv, "#version 100");

	/* Specify GL_OES_EGL_image_external */
	egl_.pushEnv(shaderEnv, "#extension GL_OES_EGL_image_external: enable");

	/*
	 * Tell shaders how to re-order output taking account of how the
	 * pixels are actually stored by GBM
	 */
	switch (outputFormat) {
	case formats::ARGB8888:
	case formats::XRGB8888:
		break;
	case formats::ABGR8888:
	case formats::XBGR8888:
		egl_.pushEnv(shaderEnv, "#define SWAP_BLUE");
		break;
	default:
		LOG(Debayer, Error) << "Unsupported output format";
		return -EINVAL;
	}

	/* Pixel location parameters */
	glFormat_ = GL_LUMINANCE;
	bytesPerPixel_ = 1;
	shaderStridePixels_ = inputConfig_.stride;

	switch (inputFormat) {
	case libcamera::formats::SBGGR8:
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SBGGR12_CSI2P:
		firstRed_x_ = 1.0;
		firstRed_y_ = 1.0;
		break;
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
		firstRed_x_ = 0.0;
		firstRed_y_ = 1.0;
		break;
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
		firstRed_x_ = 1.0;
		firstRed_y_ = 0.0;
		break;
	case libcamera::formats::SRGGB8:
	case libcamera::formats::SRGGB10_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		firstRed_x_ = 0.0;
		firstRed_y_ = 0.0;
		break;
	default:
		LOG(Debayer, Error) << "Unsupported input format";
		return -EINVAL;
	};

	/* Shader selection */
	switch (inputFormat) {
	case libcamera::formats::SBGGR8:
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SRGGB8:
		fragmentShaderData = bayer_unpacked_frag;
		fragmentShaderDataLen = bayer_unpacked_frag_len;
		vertexShaderData = bayer_unpacked_vert;
		vertexShaderDataLen = bayer_unpacked_vert_len;
		break;
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SRGGB10_CSI2P:
		egl_.pushEnv(shaderEnv, "#define RAW10P");
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::None) {
			fragmentShaderData = bayer_unpacked_frag;
			fragmentShaderDataLen = bayer_unpacked_frag_len;
			vertexShaderData = bayer_unpacked_vert;
			vertexShaderDataLen = bayer_unpacked_vert_len;
			glFormat_ = GL_RG;
			bytesPerPixel_ = 2;
		} else {
			fragmentShaderData = bayer_1x_packed_frag;
			fragmentShaderDataLen = bayer_1x_packed_frag_len;
			vertexShaderData = identity_vert;
			vertexShaderDataLen = identity_vert_len;
			shaderStridePixels_ = width_;
		}
		break;
	case libcamera::formats::SBGGR12_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		egl_.pushEnv(shaderEnv, "#define RAW12P");
		if (BayerFormat::fromPixelFormat(inputFormat).packing == BayerFormat::Packing::None) {
			fragmentShaderData = bayer_unpacked_frag;
			fragmentShaderDataLen = bayer_unpacked_frag_len;
			vertexShaderData = bayer_unpacked_vert;
			vertexShaderDataLen = bayer_unpacked_vert_len;
			glFormat_ = GL_RG;
			bytesPerPixel_ = 2;
		} else {
			fragmentShaderData = bayer_1x_packed_frag;
			fragmentShaderDataLen = bayer_1x_packed_frag_len;
			vertexShaderData = identity_vert;
			vertexShaderDataLen = identity_vert_len;
			shaderStridePixels_ = width_;
		}
		break;
	};

	if (egl_.compileVertexShader(vertexShaderId_, vertexShaderData, vertexShaderDataLen, shaderEnv)) {
		LOG(Debayer, Error) << "Compile vertex shader fail";
		return -ENODEV;
	}
	utils::scope_exit vShaderGuard([&] { glDeleteShader(vertexShaderId_); });

	if (egl_.compileFragmentShader(fragmentShaderId_, fragmentShaderData, fragmentShaderDataLen, shaderEnv)) {
		LOG(Debayer, Error) << "Compile fragment shader fail";
		return -ENODEV;
	}
	utils::scope_exit fShaderGuard([&] { glDeleteShader(fragmentShaderId_); });

	if (egl_.linkProgram(programId_, vertexShaderId_, fragmentShaderId_)) {
		LOG(Debayer, Error) << "Linking program fail";
		return -ENODEV;
	}

	egl_.dumpShaderSource(vertexShaderId_);
	egl_.dumpShaderSource(fragmentShaderId_);

	/* Ensure we set the programId_ */
	egl_.useProgram(programId_);
	err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(Debayer, Error) << "Use program error " << err;
		return -ENODEV;
	}

	return getShaderVariableLocations();
}

/**
 * \brief Get the output frame size
 *
 * \return The output frame size
 */
unsigned int DebayerEGL::frameSize()
{
	return outputConfig_.frameSize;
}

int DebayerEGL::configure(const StreamConfiguration &inputCfg,
			  const std::vector<std::reference_wrapper<StreamConfiguration>> &outputCfgs,
			  bool ccmEnabled)
{
	if (getInputConfig(inputCfg.pixelFormat, inputConfig_) != 0)
		return -EINVAL;

	if (stats_->configure(inputCfg) != 0)
		return -EINVAL;

	if (!ccmEnabled)
		return -EINVAL;

	const Size &stats_pattern_size = stats_->patternSize();
	if (inputConfig_.patternSize.width != stats_pattern_size.width ||
	    inputConfig_.patternSize.height != stats_pattern_size.height) {
		LOG(Debayer, Error)
			<< "mismatching stats and debayer pattern sizes for "
			<< inputCfg.pixelFormat;
		return -EINVAL;
	}

	inputConfig_.stride = inputCfg.stride;
	inputPixelFormat_ = inputCfg.pixelFormat;
	width_ = inputCfg.size.width;
	height_ = inputCfg.size.height;

	if (outputCfgs.size() != 1) {
		LOG(Debayer, Error)
			<< "Unsupported number of output streams: "
			<< outputCfgs.size();
		return -EINVAL;
	}

	StreamConfiguration &outputCfg = outputCfgs[0];
	SizeRange outSizeRange = sizes(inputCfg.pixelFormat, inputCfg.size);
	std::tie(outputConfig_.stride, outputConfig_.frameSize) =
		strideAndFrameSize(outputCfg.pixelFormat, outputCfg.size);

	if (!outSizeRange.contains(outputCfg.size) || outputConfig_.stride != outputCfg.stride) {
		LOG(Debayer, Error)
			<< "Invalid output size/stride: "
			<< "\n  " << outputCfg.size << " (" << outSizeRange << ")"
			<< "\n  " << outputCfg.stride << " (" << outputConfig_.stride << ")";
		return -EINVAL;
	}

	outputPixelFormat_ = outputCfg.pixelFormat;
	outputSize_ = outputCfg.size;

	window_.x = ((inputCfg.size.width - outputCfg.size.width) / 2) &
		    ~(inputConfig_.patternSize.width - 1);
	window_.y = ((inputCfg.size.height - outputCfg.size.height) / 2) &
		    ~(inputConfig_.patternSize.height - 1);
	window_.width = outputCfg.size.width;
	window_.height = outputCfg.size.height;

	/*
	 * Don't pass x,y from window_ since process() already adjusts for it.
	 * But crop the window to 2/3 of its width and height for speedup.
	 */
	stats_->setWindow(Rectangle(window_.size()));

	return 0;
}

Size DebayerEGL::patternSize(PixelFormat inputFormat)
{
	DebayerEGL::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return {};

	return config.patternSize;
}

std::vector<PixelFormat> DebayerEGL::formats(PixelFormat inputFormat)
{
	DebayerEGL::DebayerInputConfig config;

	if (getInputConfig(inputFormat, config) != 0)
		return std::vector<PixelFormat>();

	return config.outputFormats;
}

std::tuple<unsigned int, unsigned int>
DebayerEGL::strideAndFrameSize(const PixelFormat &outputFormat, const Size &size)
{
	DebayerEGL::DebayerOutputConfig config;

	if (getOutputConfig(outputFormat, config) != 0)
		return std::make_tuple(0, 0);

	/* Align stride to 256 bytes as a generic GPU memory access alignment */
	unsigned int stride = libcamera::utils::alignUp(size.width * config.bpp / 8, 256);

	return std::make_tuple(stride, stride * size.height);
}

void DebayerEGL::setShaderVariableValues(DebayerParams &params)
{
	/*
	 * Raw Bayer 8-bit, and packed raw Bayer 10-bit/12-bit formats
	 * are stored in a GL_LUMINANCE texture. The texture width is
	 * equal to the stride.
	 */
	GLfloat firstRed[] = { firstRed_x_, firstRed_y_ };
	GLfloat imgSize[] = { (GLfloat)width_,
			      (GLfloat)height_ };
	GLfloat Step[] = { static_cast<float>(bytesPerPixel_) / (inputConfig_.stride - 1),
			   1.0f / (height_ - 1) };
	GLfloat Stride = (GLfloat)width_ / (shaderStridePixels_ / bytesPerPixel_);
	/*
	 * Scale input to output size, keeping the aspect ratio and preferring
	 * cropping over black bars.
	 */
	GLfloat scale = std::max((GLfloat)window_.width / width_,
				 (GLfloat)window_.height / height_);
	GLfloat trans = -(1.0f - scale);
	GLfloat projMatrix[] = {
		scale, 0, 0, 0,
		0, scale, 0, 0,
		0, 0, 1, 0,
		trans, trans, 0, 1
	};
	/* Static const coordinates */
	static const GLfloat vcoordinates[4][2] = {
		{ -1.0f, -1.0f },
		{ -1.0f, +1.0f },
		{ +1.0f, +1.0f },
		{ +1.0f, -1.0f },
	};
	static const GLfloat tcoordinates[4][2] = {
		{ 0.0f, 0.0f },
		{ 0.0f, 1.0f },
		{ 1.0f, 1.0f },
		{ 1.0f, 0.0f },
	};

	/* vertexIn - bayer_8.vert */
	glEnableVertexAttribArray(attributeVertex_);
	glVertexAttribPointer(attributeVertex_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), vcoordinates);

	/* textureIn - bayer_8.vert */
	glEnableVertexAttribArray(attributeTexture_);
	glVertexAttribPointer(attributeTexture_, 2, GL_FLOAT, GL_TRUE,
			      2 * sizeof(GLfloat), tcoordinates);

	/*
	 * Set the sampler2D to the respective texture unit for each texutre
	 * To simultaneously sample multiple textures we need to use multiple
	 * texture units
	 */
	glUniform1i(textureUniformBayerDataIn_, eglImageBayerIn_->texture_unit_uniform_id_);

	/*
	 * These values are:
	 * firstRed = tex_bayer_first_red - bayer_8.vert
	 * imgSize = tex_size - bayer_8.vert
	 * step = tex_step - bayer_8.vert
	 * Stride = stride_factor identity.vert
	 * textureUniformProjMatri = No scaling
	 */
	glUniform2fv(textureUniformBayerFirstRed_, 1, firstRed);
	glUniform2fv(textureUniformSize_, 1, imgSize);
	glUniform2fv(textureUniformStep_, 1, Step);
	glUniform1f(textureUniformStrideFactor_, Stride);
	glUniformMatrix4fv(textureUniformProjMatrix_, 1, GL_FALSE, projMatrix);

	LOG(Debayer, Debug) << "vertexIn " << attributeVertex_ << " textureIn " << attributeTexture_
			    << " tex_y " << textureUniformBayerDataIn_
			    << " tex_step " << textureUniformStep_
			    << " tex_size " << textureUniformSize_
			    << " stride_factor " << textureUniformStrideFactor_
			    << " tex_bayer_first_red " << textureUniformBayerFirstRed_;

	LOG(Debayer, Debug) << "textureUniformY_ = 0 "
			    << " firstRed.x " << firstRed[0]
			    << " firstRed.y " << firstRed[1]
			    << " textureUniformSize_.width " << imgSize[0]
			    << " textureUniformSize_.height " << imgSize[1]
			    << " textureUniformStep_.x " << Step[0]
			    << " textureUniformStep_.y " << Step[1]
			    << " textureUniformStrideFactor_ " << Stride
			    << " textureUniformProjMatrix_ " << textureUniformProjMatrix_;

	GLfloat ccm[9] = {
		params.ccm[0][0],
		params.ccm[0][1],
		params.ccm[0][2],
		params.ccm[1][0],
		params.ccm[1][1],
		params.ccm[1][2],
		params.ccm[2][0],
		params.ccm[2][1],
		params.ccm[2][2],
	};
	glUniformMatrix3fv(ccmUniformDataIn_, 1, GL_FALSE, ccm);
	LOG(Debayer, Debug) << " ccmUniformDataIn_ " << ccmUniformDataIn_ << " data " << params.ccm;

	/*
	 * 0 = Red, 1 = Green, 2 = Blue
	 */
	glUniform3f(blackLevelUniformDataIn_, params.blackLevel[0], params.blackLevel[1], params.blackLevel[2]);
	LOG(Debayer, Debug) << " blackLevelUniformDataIn_ " << blackLevelUniformDataIn_ << " data " << params.blackLevel;

	/*
	 * Gamma
	 */
	glUniform1f(gammaUniformDataIn_, params.gamma);
	LOG(Debayer, Debug) << " gammaUniformDataIn_ " << gammaUniformDataIn_ << " data " << params.gamma;

	/*
	 * Contrast
	 */
	glUniform1f(contrastExpUniformDataIn_, params.contrastExp);
	LOG(Debayer, Debug) << " contrastExpUniformDataIn_ " << contrastExpUniformDataIn_ << " data " << params.contrastExp;

	return;
}

int DebayerEGL::debayerGPU(MappedFrameBuffer &in, int out_fd, DebayerParams &params)
{
	/* eGL context switch */
	egl_.makeCurrent();

	/* Create a standard texture input */
	egl_.createTexture2D(*eglImageBayerIn_, glFormat_, inputConfig_.stride / bytesPerPixel_, height_, in.planes()[0].data());

	/* Generate the output render framebuffer as render to texture */
	egl_.createOutputDMABufTexture2D(*eglImageBayerOut_, out_fd);

	setShaderVariableValues(params);
	glViewport(0, 0, width_, height_);
	glClear(GL_COLOR_BUFFER_BIT);
	glDrawArrays(GL_TRIANGLE_FAN, 0, DEBAYER_OPENGL_COORDS);

	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		LOG(eGL, Error) << "Drawing scene fail " << err;
		return -ENODEV;
	} else {
		egl_.syncOutput();
	}

	return 0;
}

void DebayerEGL::process(uint32_t frame, FrameBuffer *input, FrameBuffer *output, DebayerParams params)
{
	bench_.startFrame();

	std::vector<DmaSyncer> dmaSyncers;

	dmaSyncBegin(dmaSyncers, input, nullptr);

	setParams(params);

	/* Copy metadata from the input buffer */
	FrameMetadata &metadata = output->_d()->metadata();
	metadata.status = input->metadata().status;
	metadata.sequence = input->metadata().sequence;
	metadata.timestamp = input->metadata().timestamp;

	MappedFrameBuffer in(input, MappedFrameBuffer::MapFlag::Read);
	if (!in.isValid()) {
		LOG(Debayer, Error) << "mmap-ing buffer(s) failed";
		goto error;
	}

	if (debayerGPU(in, output->planes()[0].fd.get(), params)) {
		LOG(Debayer, Error) << "debayerGPU failed";
		goto error;
	}

	bench_.finishFrame();

	metadata.planes()[0].bytesused = output->planes()[0].length;

	/* Calculate stats for the whole frame */
	stats_->processFrame(frame, 0, input);
	dmaSyncers.clear();

	outputBufferReady.emit(output);
	inputBufferReady.emit(input);

	return;

error:
	bench_.finishFrame();
	metadata.status = FrameMetadata::FrameError;
	return;
}

int DebayerEGL::start()
{
	GLint maxTextureImageUnits;

	if (gbmSurface_.createDevice())
		return -ENODEV;

	if (egl_.initEGLContext(&gbmSurface_))
		return -ENODEV;

	glGetIntegerv(GL_MAX_TEXTURE_IMAGE_UNITS, &maxTextureImageUnits);

	LOG(Debayer, Debug) << "Available fragment shader texture units " << maxTextureImageUnits;

	/* Raw bayer input as texture */
	eglImageBayerIn_ = std::make_unique<eGLImage>(width_, height_, 32, inputConfig_.stride, GL_TEXTURE0, 0);

	/* Texture we will render to */
	eglImageBayerOut_ = std::make_unique<eGLImage>(outputSize_.width, outputSize_.height, 31, outputConfig_.stride, GL_TEXTURE1, 1);

	if (initBayerShaders(inputPixelFormat_, outputPixelFormat_))
		return -EINVAL;

	return 0;
}

void DebayerEGL::stop()
{
	eglImageBayerOut_.reset();
	eglImageBayerIn_.reset();

	if (programId_)
		glDeleteProgram(programId_);
}

SizeRange DebayerEGL::sizes(PixelFormat inputFormat, const Size &inputSize)
{
	Size patternSize = this->patternSize(inputFormat);
	unsigned int borderHeight = patternSize.height;

	if (patternSize.isNull())
		return {};

	/* No need for top/bottom border with a pattern height of 2 */
	if (patternSize.height == 2)
		borderHeight = 0;

	/*
	 * For debayer interpolation a border is kept around the entire image
	 * and the minimum output size is pattern-height x pattern-width.
	 */
	if (inputSize.width < (3 * patternSize.width) ||
	    inputSize.height < (2 * borderHeight + patternSize.height)) {
		LOG(Debayer, Warning)
			<< "Input format size too small: " << inputSize;
		return {};
	}

	return SizeRange(Size(patternSize.width, patternSize.height),
			 Size((inputSize.width - 2 * patternSize.width) & ~(patternSize.width - 1),
			      (inputSize.height - 2 * borderHeight) & ~(patternSize.height - 1)),
			 patternSize.width, patternSize.height);
}

} /* namespace libcamera */
