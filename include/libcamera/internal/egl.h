/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#pragma once

#include <sys/types.h>
#include <unistd.h>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>
#include <libcamera/base/utils.h>

/*
 * Workaround for build issues on Mesa <= 22.2, see
 * https://github.com/KhronosGroup/EGL-Registry/pull/130
 */
#define EGL_NO_X11

#define EGL_EGLEXT_PROTOTYPES
#include <EGL/egl.h>
#include <EGL/eglext.h>
#define GL_GLEXT_PROTOTYPES
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(eGL)

/**
 * \class eGLImage
 * \brief Helper class for managing EGL image resources
 *
 * The eGLImage class encapsulates OpenGL ES texture and framebuffer objects
 * along with their associated EGL image. It aggregates handles, descriptors,
 * and routines for managing textures that can be associated with shader
 * uniform IDs.
 *
 * This class is particularly useful for managing DMA-BUF backed textures
 * in zero-copy rendering pipelines, where textures are bound to specific
 * texture units and can be used as both input textures and render targets.
 */
class eGLImage
{
public:
	/**
	 * \brief Construct an eGLImage with explicit stride
	 * \param[in] width Image width in pixels
	 * \param[in] height Image height in pixels
	 * \param[in] stride Row stride in bytes
	 * \param[in] texture_unit OpenGL texture unit
	 * \param[in] texture_unit_uniform_id Shader uniform ID
	 */
	eGLImage(uint32_t width, uint32_t height, uint32_t stride, GLenum texture_unit, uint32_t texture_unit_uniform_id)
		: width_(width), height_(height), stride_(stride),
		  framesize_(stride * height),
		  texture_unit_uniform_id_(texture_unit_uniform_id),
		  texture_unit_(texture_unit)
	{
		glGenTextures(1, &texture_);
		glGenFramebuffers(1, &fbo_);
	}

	/**
	 * \brief Destroy the eGLImage
	 *
	 * Cleans up OpenGL resources by deleting the framebuffer object and
	 * texture.
	 */
	~eGLImage()
	{
		glDeleteFramebuffers(1, &fbo_);
		glDeleteTextures(1, &texture_);
	}

	uint32_t width_; /**< Image width in pixels */
	uint32_t height_; /**< Image height in pixels */
	uint32_t stride_; /**< Row stride in bytes */
	uint32_t offset_; /**< Buffer offset (reserved for future use) */
	uint32_t framesize_; /**< Total frame size in bytes (stride * height) */
	uint32_t texture_unit_uniform_id_; /**< Shader uniform id for texture unit */
	GLenum texture_unit_; /**< Texture unit associated with this image eg (GL_TEXTURE0) */
	GLuint texture_; /**< OpenGL texture object ID */
	GLuint fbo_; /**< OpenGL frame buffer object ID */
	EGLImageKHR image_; /**< EGL Image handle */

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(eGLImage)
};

class eGL
{
public:
	eGL();
	~eGL();

	int initEGLContext();

	int createInputDMABufTexture2D(eGLImage &eglImage, int fd);
	int createOutputDMABufTexture2D(eGLImage &eglImage, int fd);
	void destroyDMABufTexture(eGLImage &eglImage);
	void createTexture2D(eGLImage &eglImage, GLint format, uint32_t width, uint32_t height, void *data);

	void pushEnv(std::vector<std::string> &shaderEnv, const char *str);
	void makeCurrent();

	int compileVertexShader(GLuint &shaderId, const unsigned char *shaderData,
				unsigned int shaderDataLen,
				Span<const std::string> shaderEnv);
	int compileFragmentShader(GLuint &shaderId, const unsigned char *shaderData,
				  unsigned int shaderDataLen,
				  Span<const std::string> shaderEnv);
	int linkProgram(GLuint &programId, GLuint fragmentshaderId, GLuint vertexshaderId);
	void dumpShaderSource(GLuint shaderId);
	void useProgram(GLuint programId);
	void deleteProgram(GLuint programId);
	void syncOutput();

private:
	LIBCAMERA_DISABLE_COPY_AND_MOVE(eGL)

	pid_t tid_;

	EGLDisplay display_ = EGL_NO_DISPLAY;
	EGLContext context_ = EGL_NO_CONTEXT;
	EGLSurface surface_ = EGL_NO_SURFACE;

	int compileShader(int shaderType, GLuint &shaderId, const unsigned char *shaderData,
			  unsigned int shaderDataLen,
			  Span<const std::string> shaderEnv);

	int createDMABufTexture2D(eGLImage &eglImage, int fd, bool output);

	PFNGLEGLIMAGETARGETTEXTURE2DOESPROC glEGLImageTargetTexture2DOES;
	PFNEGLCREATEIMAGEKHRPROC eglCreateImageKHR;
	PFNEGLDESTROYIMAGEKHRPROC eglDestroyImageKHR;
	PFNGLGETSTRINGPROC glGetString;
};
} //namespace libcamera
