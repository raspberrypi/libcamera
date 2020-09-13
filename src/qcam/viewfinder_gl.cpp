/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * viewfinderGL.cpp - OpenGL Viewfinder for rendering by OpenGL shader
 */

#include "viewfinder_gl.h"

#include <QImage>

#include <libcamera/formats.h>

static const QList<libcamera::PixelFormat> supportedFormats{
	libcamera::formats::NV12,
	libcamera::formats::NV21,
	libcamera::formats::NV16,
	libcamera::formats::NV61,
	libcamera::formats::NV24,
	libcamera::formats::NV42,
	libcamera::formats::YUV420,
	libcamera::formats::YVU420,
};

ViewFinderGL::ViewFinderGL(QWidget *parent)
	: QOpenGLWidget(parent), buffer_(nullptr), yuvData_(nullptr),
	  fragmentShader_(nullptr), vertexShader_(nullptr),
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
	/* If the fragment is created remove it and create a new one. */
	if (fragmentShader_) {
		if (shaderProgram_.isLinked()) {
			shaderProgram_.release();
			shaderProgram_.removeShader(fragmentShader_);
			delete fragmentShader_;
		}
	}

	if (!selectFormat(format))
		return -1;

	format_ = format;
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
	switch (format) {
	case libcamera::formats::NV12:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_2_planes_UV_f.glsl";
		break;
	case libcamera::formats::NV21:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_2_planes_VU_f.glsl";
		break;
	case libcamera::formats::NV16:
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_2_planes_UV_f.glsl";
		break;
	case libcamera::formats::NV61:
		horzSubSample_ = 2;
		vertSubSample_ = 1;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_2_planes_VU_f.glsl";
		break;
	case libcamera::formats::NV24:
		horzSubSample_ = 1;
		vertSubSample_ = 1;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_2_planes_UV_f.glsl";
		break;
	case libcamera::formats::NV42:
		horzSubSample_ = 1;
		vertSubSample_ = 1;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_2_planes_VU_f.glsl";
		break;
	case libcamera::formats::YUV420:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_3_planes_f.glsl";
		break;
	case libcamera::formats::YVU420:
		horzSubSample_ = 2;
		vertSubSample_ = 2;
		vertexShaderSrc_ = ":NV_vertex_shader.glsl";
		fragmentShaderSrc_ = ":NV_3_planes_f.glsl";
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
	vertexShader_ = new QOpenGLShader(QOpenGLShader::Vertex, this);

	/* Compile the vertex shader */
	if (!vertexShader_->compileSourceFile(vertexShaderSrc_)) {
		qWarning() << "[ViewFinderGL]:" << vertexShader_->log();
		return false;
	}

	shaderProgram_.addShader(vertexShader_);
	return true;
}

bool ViewFinderGL::createFragmentShader()
{
	int attributeVertex;
	int attributeTexture;

	/* Create Fragment Shader */
	fragmentShader_ = new QOpenGLShader(QOpenGLShader::Fragment, this);

	/* Compile the fragment shader */
	if (!fragmentShader_->compileSourceFile(fragmentShaderSrc_)) {
		qWarning() << "[ViewFinderGL]:" << fragmentShader_->log();
		return false;
	}

	shaderProgram_.addShader(fragmentShader_);

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

	if (fragmentShader_)
		delete fragmentShader_;

	if (vertexShader_)
		delete vertexShader_;
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
			     (char *)yuvData_ + size_.width() * size_.height());
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
			     (char *)yuvData_ + size_.width() * size_.height());
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
			     (char *)yuvData_ + size_.width() * size_.height() * 5 / 4);
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
			     (char *)yuvData_ + size_.width() * size_.height());
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
			     (char *)yuvData_ + size_.width() * size_.height() * 5 / 4);
		shaderProgram_.setUniformValue(textureUniformU_, 1);
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
