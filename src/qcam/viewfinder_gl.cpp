/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * viewfinderGL.cpp - OpenGL Viewfinder for rendering by OpenGL shader
 */

#include "viewfinder_gl.h"

#include <QByteArray>
#include <QFile>
#include <QImage>

#include <libcamera/formats.h>

static const QList<libcamera::PixelFormat> supportedFormats{
	/* YUV - packed (single plane) */
	libcamera::formats::UYVY,
	libcamera::formats::VYUY,
	libcamera::formats::YUYV,
	libcamera::formats::YVYU,
	/* YUV - semi planar (two planes) */
	libcamera::formats::NV12,
	libcamera::formats::NV21,
	libcamera::formats::NV16,
	libcamera::formats::NV61,
	libcamera::formats::NV24,
	libcamera::formats::NV42,
	/* YUV - fully planar (three planes) */
	libcamera::formats::YUV420,
	libcamera::formats::YVU420,
	/* RGB */
	libcamera::formats::ABGR8888,
	libcamera::formats::ARGB8888,
	libcamera::formats::BGRA8888,
	libcamera::formats::RGBA8888,
	libcamera::formats::BGR888,
	libcamera::formats::RGB888,
	/* Raw Bayer 8-bit */
	libcamera::formats::SBGGR8,
	libcamera::formats::SGBRG8,
	libcamera::formats::SGRBG8,
	libcamera::formats::SRGGB8,
	/* Raw Bayer 10-bit packed */
	libcamera::formats::SBGGR10_CSI2P,
	libcamera::formats::SGBRG10_CSI2P,
	libcamera::formats::SGRBG10_CSI2P,
	libcamera::formats::SRGGB10_CSI2P,
	/* Raw Bayer 12-bit packed */
	libcamera::formats::SBGGR12_CSI2P,
	libcamera::formats::SGBRG12_CSI2P,
	libcamera::formats::SGRBG12_CSI2P,
	libcamera::formats::SRGGB12_CSI2P,
};

ViewFinderGL::ViewFinderGL(QWidget *parent)
	: QOpenGLWidget(parent), buffer_(nullptr), data_(nullptr),
	  vertexBuffer_(QOpenGLBuffer::VertexBuffer)
{
}

ViewFinderGL::~ViewFinderGL()
{
	removeShader();
}

const QList<libcamera::PixelFormat> &ViewFinderGL::nativeFormats() const
{
	return supportedFormats;
}

int ViewFinderGL::setFormat(const libcamera::PixelFormat &format,
			    const QSize &size)
{
	if (format != format_) {
		/*
		 * If the fragment already exists, remove it and create a new
		 * one for the new format.
		 */
		if (shaderProgram_.isLinked()) {
			shaderProgram_.release();
			shaderProgram_.removeShader(fragmentShader_.get());
			fragmentShader_.reset();
		}

		if (!selectFormat(format))
			return -1;

		format_ = format;
	}

	size_ = size;

	updateGeometry();
	return 0;
}

void ViewFinderGL::stop()
{
	if (buffer_) {
		renderComplete(buffer_);
		buffer_ = nullptr;
	}
}

QImage ViewFinderGL::getCurrentImage()
{
	QMutexLocker locker(&mutex_);

	return grabFramebuffer();
}

void ViewFinderGL::render(libcamera::FrameBuffer *buffer, MappedBuffer *map)
{
	if (buffer->planes().size() != 1) {
		qWarning() << "Multi-planar buffers are not supported";
		return;
	}

	if (buffer_)
		renderComplete(buffer_);

	data_ = static_cast<unsigned char *>(map->memory);
	/*
	 * \todo Get the stride from the buffer instead of computing it naively
	 */
	stride_ = buffer->metadata().planes[0].bytesused / size_.height();
	update();
	buffer_ = buffer;
}

bool ViewFinderGL::selectFormat(const libcamera::PixelFormat &format)
{
	bool ret = true;

	/* Set min/mag filters to GL_LINEAR by default. */
	textureMinMagFilters_ = GL_LINEAR;

	/* Use identity.vert as the default vertex shader. */
	vertexShaderFile_ = ":identity.vert";

	fragmentShaderDefines_.clear();

	switch (format) {
	case libcamera::formats::NV12:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		fragmentShaderDefines_.append("#define YUV_PATTERN_UV");
		fragmentShaderFile_ = ":YUV_2_planes.frag";
		break;
	case libcamera::formats::NV21:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		fragmentShaderDefines_.append("#define YUV_PATTERN_VU");
		fragmentShaderFile_ = ":YUV_2_planes.frag";
		break;
	case libcamera::formats::NV16:
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		fragmentShaderDefines_.append("#define YUV_PATTERN_UV");
		fragmentShaderFile_ = ":YUV_2_planes.frag";
		break;
	case libcamera::formats::NV61:
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		fragmentShaderDefines_.append("#define YUV_PATTERN_VU");
		fragmentShaderFile_ = ":YUV_2_planes.frag";
		break;
	case libcamera::formats::NV24:
		horzSubSample_ = 1;
		vertSubSample_ = 1;
		fragmentShaderDefines_.append("#define YUV_PATTERN_UV");
		fragmentShaderFile_ = ":YUV_2_planes.frag";
		break;
	case libcamera::formats::NV42:
		horzSubSample_ = 1;
		vertSubSample_ = 1;
		fragmentShaderDefines_.append("#define YUV_PATTERN_VU");
		fragmentShaderFile_ = ":YUV_2_planes.frag";
		break;
	case libcamera::formats::YUV420:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		fragmentShaderFile_ = ":YUV_3_planes.frag";
		break;
	case libcamera::formats::YVU420:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		fragmentShaderFile_ = ":YUV_3_planes.frag";
		break;
	case libcamera::formats::UYVY:
		fragmentShaderDefines_.append("#define YUV_PATTERN_UYVY");
		fragmentShaderFile_ = ":YUV_packed.frag";
		break;
	case libcamera::formats::VYUY:
		fragmentShaderDefines_.append("#define YUV_PATTERN_VYUY");
		fragmentShaderFile_ = ":YUV_packed.frag";
		break;
	case libcamera::formats::YUYV:
		fragmentShaderDefines_.append("#define YUV_PATTERN_YUYV");
		fragmentShaderFile_ = ":YUV_packed.frag";
		break;
	case libcamera::formats::YVYU:
		fragmentShaderDefines_.append("#define YUV_PATTERN_YVYU");
		fragmentShaderFile_ = ":YUV_packed.frag";
		break;
	case libcamera::formats::ABGR8888:
		fragmentShaderDefines_.append("#define RGB_PATTERN rgb");
		fragmentShaderFile_ = ":RGB.frag";
		break;
	case libcamera::formats::ARGB8888:
		fragmentShaderDefines_.append("#define RGB_PATTERN bgr");
		fragmentShaderFile_ = ":RGB.frag";
		break;
	case libcamera::formats::BGRA8888:
		fragmentShaderDefines_.append("#define RGB_PATTERN gba");
		fragmentShaderFile_ = ":RGB.frag";
		break;
	case libcamera::formats::RGBA8888:
		fragmentShaderDefines_.append("#define RGB_PATTERN abg");
		fragmentShaderFile_ = ":RGB.frag";
		break;
	case libcamera::formats::BGR888:
		fragmentShaderDefines_.append("#define RGB_PATTERN rgb");
		fragmentShaderFile_ = ":RGB.frag";
		break;
	case libcamera::formats::RGB888:
		fragmentShaderDefines_.append("#define RGB_PATTERN bgr");
		fragmentShaderFile_ = ":RGB.frag";
		break;
	case libcamera::formats::SBGGR8:
		firstRed_.setX(1.0);
		firstRed_.setY(1.0);
		vertexShaderFile_ = ":bayer_8.vert";
		fragmentShaderFile_ = ":bayer_8.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SGBRG8:
		firstRed_.setX(0.0);
		firstRed_.setY(1.0);
		vertexShaderFile_ = ":bayer_8.vert";
		fragmentShaderFile_ = ":bayer_8.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SGRBG8:
		firstRed_.setX(1.0);
		firstRed_.setY(0.0);
		vertexShaderFile_ = ":bayer_8.vert";
		fragmentShaderFile_ = ":bayer_8.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SRGGB8:
		firstRed_.setX(0.0);
		firstRed_.setY(0.0);
		vertexShaderFile_ = ":bayer_8.vert";
		fragmentShaderFile_ = ":bayer_8.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SBGGR10_CSI2P:
		firstRed_.setX(1.0);
		firstRed_.setY(1.0);
		fragmentShaderDefines_.append("#define RAW10P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SGBRG10_CSI2P:
		firstRed_.setX(0.0);
		firstRed_.setY(1.0);
		fragmentShaderDefines_.append("#define RAW10P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SGRBG10_CSI2P:
		firstRed_.setX(1.0);
		firstRed_.setY(0.0);
		fragmentShaderDefines_.append("#define RAW10P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SRGGB10_CSI2P:
		firstRed_.setX(0.0);
		firstRed_.setY(0.0);
		fragmentShaderDefines_.append("#define RAW10P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SBGGR12_CSI2P:
		firstRed_.setX(1.0);
		firstRed_.setY(1.0);
		fragmentShaderDefines_.append("#define RAW12P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SGBRG12_CSI2P:
		firstRed_.setX(0.0);
		firstRed_.setY(1.0);
		fragmentShaderDefines_.append("#define RAW12P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SGRBG12_CSI2P:
		firstRed_.setX(1.0);
		firstRed_.setY(0.0);
		fragmentShaderDefines_.append("#define RAW12P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	case libcamera::formats::SRGGB12_CSI2P:
		firstRed_.setX(0.0);
		firstRed_.setY(0.0);
		fragmentShaderDefines_.append("#define RAW12P");
		fragmentShaderFile_ = ":bayer_1x_packed.frag";
		textureMinMagFilters_ = GL_NEAREST;
		break;
	default:
		ret = false;
		qWarning() << "[ViewFinderGL]:"
			   << "format not supported.";
		break;
	};

	return ret;
}

bool ViewFinderGL::createVertexShader()
{
	/* Create Vertex Shader */
	vertexShader_ = std::make_unique<QOpenGLShader>(QOpenGLShader::Vertex, this);

	/* Compile the vertex shader */
	if (!vertexShader_->compileSourceFile(vertexShaderFile_)) {
		qWarning() << "[ViewFinderGL]:" << vertexShader_->log();
		return false;
	}

	shaderProgram_.addShader(vertexShader_.get());
	return true;
}

bool ViewFinderGL::createFragmentShader()
{
	int attributeVertex;
	int attributeTexture;

	/*
	 * Create the fragment shader, compile it, and add it to the shader
	 * program. The #define macros stored in fragmentShaderDefines_, if
	 * any, are prepended to the source code.
	 */
	fragmentShader_ = std::make_unique<QOpenGLShader>(QOpenGLShader::Fragment, this);

	QFile file(fragmentShaderFile_);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		qWarning() << "Shader" << fragmentShaderFile_ << "not found";
		return false;
	}

	QString defines = fragmentShaderDefines_.join('\n') + "\n";
	QByteArray src = file.readAll();
	src.prepend(defines.toUtf8());

	if (!fragmentShader_->compileSourceCode(src)) {
		qWarning() << "[ViewFinderGL]:" << fragmentShader_->log();
		return false;
	}

	shaderProgram_.addShader(fragmentShader_.get());

	/* Link shader pipeline */
	if (!shaderProgram_.link()) {
		qWarning() << "[ViewFinderGL]:" << shaderProgram_.log();
		close();
	}

	/* Bind shader pipeline for use */
	if (!shaderProgram_.bind()) {
		qWarning() << "[ViewFinderGL]:" << shaderProgram_.log();
		close();
	}

	attributeVertex = shaderProgram_.attributeLocation("vertexIn");
	attributeTexture = shaderProgram_.attributeLocation("textureIn");

	shaderProgram_.enableAttributeArray(attributeVertex);
	shaderProgram_.setAttributeBuffer(attributeVertex,
					  GL_FLOAT,
					  0,
					  2,
					  2 * sizeof(GLfloat));

	shaderProgram_.enableAttributeArray(attributeTexture);
	shaderProgram_.setAttributeBuffer(attributeTexture,
					  GL_FLOAT,
					  8 * sizeof(GLfloat),
					  2,
					  2 * sizeof(GLfloat));

	textureUniformY_ = shaderProgram_.uniformLocation("tex_y");
	textureUniformU_ = shaderProgram_.uniformLocation("tex_u");
	textureUniformV_ = shaderProgram_.uniformLocation("tex_v");
	textureUniformStep_ = shaderProgram_.uniformLocation("tex_step");
	textureUniformSize_ = shaderProgram_.uniformLocation("tex_size");
	textureUniformBayerFirstRed_ = shaderProgram_.uniformLocation("tex_bayer_first_red");

	/* Create the textures. */
	for (std::unique_ptr<QOpenGLTexture> &texture : textures_) {
		if (texture)
			continue;

		texture = std::make_unique<QOpenGLTexture>(QOpenGLTexture::Target2D);
		texture->create();
	}

	return true;
}

void ViewFinderGL::configureTexture(QOpenGLTexture &texture)
{
	glBindTexture(GL_TEXTURE_2D, texture.textureId());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
			textureMinMagFilters_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			textureMinMagFilters_);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void ViewFinderGL::removeShader()
{
	if (shaderProgram_.isLinked()) {
		shaderProgram_.release();
		shaderProgram_.removeAllShaders();
	}
}

void ViewFinderGL::initializeGL()
{
	initializeOpenGLFunctions();
	glEnable(GL_TEXTURE_2D);
	glDisable(GL_DEPTH_TEST);

	static const GLfloat coordinates[2][4][2]{
		{
			/* Vertex coordinates */
			{ -1.0f, -1.0f },
			{ -1.0f, +1.0f },
			{ +1.0f, +1.0f },
			{ +1.0f, -1.0f },
		},
		{
			/* Texture coordinates */
			{ 0.0f, 1.0f },
			{ 0.0f, 0.0f },
			{ 1.0f, 0.0f },
			{ 1.0f, 1.0f },
		},
	};

	vertexBuffer_.create();
	vertexBuffer_.bind();
	vertexBuffer_.allocate(coordinates, sizeof(coordinates));

	/* Create Vertex Shader */
	if (!createVertexShader())
		qWarning() << "[ViewFinderGL]: create vertex shader failed.";

	glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
}

void ViewFinderGL::doRender()
{
	switch (format_) {
	case libcamera::formats::NV12:
	case libcamera::formats::NV21:
	case libcamera::formats::NV16:
	case libcamera::formats::NV61:
	case libcamera::formats::NV24:
	case libcamera::formats::NV42:
		/* Activate texture Y */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/* Activate texture UV/VU */
		glActiveTexture(GL_TEXTURE1);
		configureTexture(*textures_[1]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RG,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RG,
			     GL_UNSIGNED_BYTE,
			     data_ + size_.width() * size_.height());
		shaderProgram_.setUniformValue(textureUniformU_, 1);
		break;

	case libcamera::formats::YUV420:
		/* Activate texture Y */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/* Activate texture U */
		glActiveTexture(GL_TEXTURE1);
		configureTexture(*textures_[1]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_ + size_.width() * size_.height());
		shaderProgram_.setUniformValue(textureUniformU_, 1);

		/* Activate texture V */
		glActiveTexture(GL_TEXTURE2);
		configureTexture(*textures_[2]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_ + size_.width() * size_.height() * 5 / 4);
		shaderProgram_.setUniformValue(textureUniformV_, 2);
		break;

	case libcamera::formats::YVU420:
		/* Activate texture Y */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/* Activate texture V */
		glActiveTexture(GL_TEXTURE2);
		configureTexture(*textures_[2]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_ + size_.width() * size_.height());
		shaderProgram_.setUniformValue(textureUniformV_, 2);

		/* Activate texture U */
		glActiveTexture(GL_TEXTURE1);
		configureTexture(*textures_[1]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_ + size_.width() * size_.height() * 5 / 4);
		shaderProgram_.setUniformValue(textureUniformU_, 1);
		break;

	case libcamera::formats::UYVY:
	case libcamera::formats::VYUY:
	case libcamera::formats::YUYV:
	case libcamera::formats::YVYU:
		/*
		 * Packed YUV formats are stored in a RGBA texture to match the
		 * OpenGL texel size with the 4 bytes repeating pattern in YUV.
		 * The texture width is thus half of the image with.
		 */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RGBA,
			     size_.width() / 2,
			     size_.height(),
			     0,
			     GL_RGBA,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/*
		 * The shader needs the step between two texture pixels in the
		 * horizontal direction, expressed in texture coordinate units
		 * ([0, 1]). There are exactly width - 1 steps between the
		 * leftmost and rightmost texels.
		 */
		shaderProgram_.setUniformValue(textureUniformStep_,
					       1.0f / (size_.width() / 2 - 1),
					       1.0f /* not used */);
		break;

	case libcamera::formats::ABGR8888:
	case libcamera::formats::ARGB8888:
	case libcamera::formats::BGRA8888:
	case libcamera::formats::RGBA8888:
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RGBA,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RGBA,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);
		break;

	case libcamera::formats::BGR888:
	case libcamera::formats::RGB888:
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RGB,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RGB,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);
		break;

	case libcamera::formats::SBGGR8:
	case libcamera::formats::SGBRG8:
	case libcamera::formats::SGRBG8:
	case libcamera::formats::SRGGB8:
	case libcamera::formats::SBGGR10_CSI2P:
	case libcamera::formats::SGBRG10_CSI2P:
	case libcamera::formats::SGRBG10_CSI2P:
	case libcamera::formats::SRGGB10_CSI2P:
	case libcamera::formats::SBGGR12_CSI2P:
	case libcamera::formats::SGBRG12_CSI2P:
	case libcamera::formats::SGRBG12_CSI2P:
	case libcamera::formats::SRGGB12_CSI2P:
		/*
		 * Raw Bayer 8-bit, and packed raw Bayer 10-bit/12-bit formats
		 * are stored in GL_RED texture.
		 * The texture width is equal to the stride.
		 */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(*textures_[0]);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     stride_,
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     data_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);
		shaderProgram_.setUniformValue(textureUniformBayerFirstRed_,
					       firstRed_);
		shaderProgram_.setUniformValue(textureUniformSize_,
					       size_.width(), /* in pixels */
					       size_.height());
		shaderProgram_.setUniformValue(textureUniformStep_,
					       1.0f / (stride_ - 1),
					       1.0f / (size_.height() - 1));
		break;

	default:
		break;
	};
}

void ViewFinderGL::paintGL()
{
	if (!fragmentShader_)
		if (!createFragmentShader()) {
			qWarning() << "[ViewFinderGL]:"
				   << "create fragment shader failed.";
		}

	if (data_) {
		glClearColor(0.0, 0.0, 0.0, 1.0);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		doRender();
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
	}
}

void ViewFinderGL::resizeGL(int w, int h)
{
	glViewport(0, 0, w, h);
}

QSize ViewFinderGL::sizeHint() const
{
	return size_.isValid() ? size_ : QSize(640, 480);
}
