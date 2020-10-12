/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Linaro
 *
 * viewfinder_GL.h - OpenGL Viewfinder for rendering by OpenGL shader
 *
 */
#ifndef __VIEWFINDER_GL_H__
#define __VIEWFINDER_GL_H__

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

#include <libcamera/buffer.h>
#include <libcamera/formats.h>

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

	int setFormat(const libcamera::PixelFormat &format, const QSize &size) override;
	void render(libcamera::FrameBuffer *buffer, MappedBuffer *map) override;
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

	void configureTexture(QOpenGLTexture &texture);
	bool createFragmentShader();
	bool createVertexShader();
	void removeShader();
	void doRender();

	/* Captured image size, format and buffer */
	libcamera::FrameBuffer *buffer_;
	libcamera::PixelFormat format_;
	QSize size_;
	unsigned char *yuvData_;

	/* Shaders */
	QOpenGLShaderProgram shaderProgram_;
	std::unique_ptr<QOpenGLShader> vertexShader_;
	std::unique_ptr<QOpenGLShader> fragmentShader_;
	QString fragmentShaderFile_;
	QStringList fragmentShaderDefines_;

	/* Vertex buffer */
	QOpenGLBuffer vertexBuffer_;

	/* YUV texture planars and parameters */
	GLuint textureUniformU_;
	GLuint textureUniformV_;
	GLuint textureUniformY_;
	GLuint textureUniformStepX_;
	QOpenGLTexture textureU_;
	QOpenGLTexture textureV_;
	QOpenGLTexture textureY_;
	unsigned int horzSubSample_;
	unsigned int vertSubSample_;

	QMutex mutex_; /* Prevent concurrent access to image_ */
};

#endif /* __VIEWFINDER_GL_H__ */
