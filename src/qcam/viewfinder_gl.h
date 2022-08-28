/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * viewfinder_GL.h - OpenGL Viewfinder for rendering by OpenGL shader
 *
 */

#pragma once

#include <array>
#include <memory>

#include <QImage>
#include <QMutex>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
#include <QOpenGLShader>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLWidget>
#include <QSize>

#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>

#include "viewfinder.h"

class ViewFinderGL : public QOpenGLWidget,
		     public ViewFinder,
		     protected QOpenGLFunctions
{
	Q_OBJECT

public:
	ViewFinderGL(QWidget *parent = nullptr);
	~ViewFinderGL();

	const QList<libcamera::PixelFormat> &nativeFormats() const override;

	int setFormat(const libcamera::PixelFormat &format, const QSize &size,
		      const libcamera::ColorSpace &colorSpace,
		      unsigned int stride) override;
	void render(libcamera::FrameBuffer *buffer, Image *image) override;
	void stop() override;

	QImage getCurrentImage() override;

Q_SIGNALS:
	void renderComplete(libcamera::FrameBuffer *buffer);

protected:
	void initializeGL() override;
	void paintGL() override;
	void resizeGL(int w, int h) override;
	QSize sizeHint() const override;

private:
	bool selectFormat(const libcamera::PixelFormat &format);
	void selectColorSpace(const libcamera::ColorSpace &colorSpace);

	void configureTexture(QOpenGLTexture &texture);
	bool createFragmentShader();
	bool createVertexShader();
	void removeShader();
	void doRender();

	/* Captured image size, format and buffer */
	libcamera::FrameBuffer *buffer_;
	libcamera::PixelFormat format_;
	libcamera::ColorSpace colorSpace_;
	QSize size_;
	unsigned int stride_;
	Image *image_;

	/* Shaders */
	QOpenGLShaderProgram shaderProgram_;
	std::unique_ptr<QOpenGLShader> vertexShader_;
	std::unique_ptr<QOpenGLShader> fragmentShader_;
	QString vertexShaderFile_;
	QString fragmentShaderFile_;
	QStringList fragmentShaderDefines_;

	/* Vertex buffer */
	QOpenGLBuffer vertexBuffer_;

	/* Textures */
	std::array<std::unique_ptr<QOpenGLTexture>, 3> textures_;

	/* Common texture parameters */
	GLuint textureMinMagFilters_;

	/* YUV texture parameters */
	GLuint textureUniformU_;
	GLuint textureUniformV_;
	GLuint textureUniformY_;
	GLuint textureUniformStep_;
	unsigned int horzSubSample_;
	unsigned int vertSubSample_;

	/* Raw Bayer texture parameters */
	GLuint textureUniformSize_;
	GLuint textureUniformStrideFactor_;
	GLuint textureUniformBayerFirstRed_;
	QPointF firstRed_;

	QMutex mutex_; /* Prevent concurrent access to image_ */
};
