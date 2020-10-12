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
	/* Packed (single plane) */
	libcamera::formats::UYVY,
	libcamera::formats::VYUY,
	libcamera::formats::YUYV,
	libcamera::formats::YVYU,
	/* Semi planar (two planes) */
	libcamera::formats::NV12,
	libcamera::formats::NV21,
	libcamera::formats::NV16,
	libcamera::formats::NV61,
	libcamera::formats::NV24,
	libcamera::formats::NV42,
	/* Fully planar (three planes) */
	libcamera::formats::YUV420,
	libcamera::formats::YVU420,
};

ViewFinderGL::ViewFinderGL(QWidget *parent)
	: QOpenGLWidget(parent), buffer_(nullptr), yuvData_(nullptr),
	  vertexBuffer_(QOpenGLBuffer::VertexBuffer),
	  textureU_(QOpenGLTexture::Target2D),
	  textureV_(QOpenGLTexture::Target2D),
	  textureY_(QOpenGLTexture::Target2D)
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

	yuvData_ = static_cast<unsigned char *>(map->memory);
	update();
	buffer_ = buffer;
}

bool ViewFinderGL::selectFormat(const libcamera::PixelFormat &format)
{
	bool ret = true;

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
	if (!vertexShader_->compileSourceFile(":identity.vert")) {
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
	textureUniformStepX_ = shaderProgram_.uniformLocation("tex_stepx");

	if (!textureY_.isCreated())
		textureY_.create();

	if (!textureU_.isCreated())
		textureU_.create();

	if (!textureV_.isCreated())
		textureV_.create();

	return true;
}

void ViewFinderGL::configureTexture(QOpenGLTexture &texture)
{
	glBindTexture(GL_TEXTURE_2D, texture.textureId());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
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
		configureTexture(textureY_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/* Activate texture UV/VU */
		glActiveTexture(GL_TEXTURE1);
		configureTexture(textureU_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RG,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RG,
			     GL_UNSIGNED_BYTE,
			     yuvData_ + size_.width() * size_.height());
		shaderProgram_.setUniformValue(textureUniformU_, 1);
		break;

	case libcamera::formats::YUV420:
		/* Activate texture Y */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(textureY_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/* Activate texture U */
		glActiveTexture(GL_TEXTURE1);
		configureTexture(textureU_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_ + size_.width() * size_.height());
		shaderProgram_.setUniformValue(textureUniformU_, 1);

		/* Activate texture V */
		glActiveTexture(GL_TEXTURE2);
		configureTexture(textureV_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_ + size_.width() * size_.height() * 5 / 4);
		shaderProgram_.setUniformValue(textureUniformV_, 2);
		break;

	case libcamera::formats::YVU420:
		/* Activate texture Y */
		glActiveTexture(GL_TEXTURE0);
		configureTexture(textureY_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width(),
			     size_.height(),
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/* Activate texture V */
		glActiveTexture(GL_TEXTURE2);
		configureTexture(textureV_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_ + size_.width() * size_.height());
		shaderProgram_.setUniformValue(textureUniformV_, 2);

		/* Activate texture U */
		glActiveTexture(GL_TEXTURE1);
		configureTexture(textureU_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RED,
			     size_.width() / horzSubSample_,
			     size_.height() / vertSubSample_,
			     0,
			     GL_RED,
			     GL_UNSIGNED_BYTE,
			     yuvData_ + size_.width() * size_.height() * 5 / 4);
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
		configureTexture(textureY_);
		glTexImage2D(GL_TEXTURE_2D,
			     0,
			     GL_RGBA,
			     size_.width() / 2,
			     size_.height(),
			     0,
			     GL_RGBA,
			     GL_UNSIGNED_BYTE,
			     yuvData_);
		shaderProgram_.setUniformValue(textureUniformY_, 0);

		/*
		 * The shader needs the step between two texture pixels in the
		 * horizontal direction, expressed in texture coordinate units
		 * ([0, 1]). There are exactly width - 1 steps between the
		 * leftmost and rightmost texels.
		 */
		shaderProgram_.setUniformValue(textureUniformStepX_,
					       1.0f / (size_.width() / 2 - 1));
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

	if (yuvData_) {
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
