/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Linaro Ltd.
 *
 * Authors:
 * Bryan O'Donoghue <bryan.odonoghue@linaro.org>
 *
 */

#include "libcamera/internal/egl.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include <libcamera/base/thread.h>

#include <libdrm/drm_fourcc.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(eGL)

/**
 * \class eGL
 * \brief Helper class for managing OpenGL ES operations
 *
 * It provides:
 *
 * - EGL context setup and management
 * - Extension function pointer retrieval
 * - Shader compilation and program linking
 * - DMA-BUF texture creation and management
 * - Synchronisation primitives
 *
 * This class is designed to work with zero-copy buffers via DMA-BUF file
 * descriptors.
 */

/**
 *\var eGL::tid_
 *\brief Thread ID of the thread associated with this EGL context
 */

/**
 *\var eGL::display_
 *\brief EGL display handle
 */

/**
 *\var eGL::context_
 *\brief EGL context handle
 */

/**
 *\var eGL::surface_
 *\brief EGL sufrace handle
 */

/**
 * \brief Construct an EGL helper
 *
 * Creates an eGL instance with uninitialised context. Call initEGLContext()
 * to set up the EGL display, context, and load extension functions.
 */
eGL::eGL()
{
}

/**
 * \brief Destroy the EGL helper
 *
 * Destroys the EGL context and surface if they were successfully created.
 */
eGL::~eGL()
{
	if (context_ != EGL_NO_CONTEXT)
		eglDestroyContext(display_, context_);

	if (surface_ != EGL_NO_SURFACE)
		eglDestroySurface(display_, surface_);
}

/**
 * \brief Synchronise rendering output
 *
 * Sychronise here. Calls glFinish() right now.
 *
 */
void eGL::syncOutput()
{
	ASSERT(tid_ == Thread::currentId());

	glFinish();
}

/**
 * \brief Create a DMA-BUF backed 2D texture
 * \param[in,out] eglImage EGL image to associate with the DMA-BUF
 * \param[in] fd DMA-BUF file descriptor
 * \param[in] output If true, create framebuffer for render target
 *
 * Internal implementation for creating DMA-BUF textures. Creates an EGL
 * image from the DMA-BUF and binds it to a 2D texture. If output is true,
 * also creates and attaches a framebuffer object.
 *
 * \return 0 on success, or -ENODEV on failure
 */
int eGL::createDMABufTexture2D(eGLImage &eglImage, int fd, bool output)
{
	int ret = 0;

	ASSERT(tid_ == Thread::currentId());

	// clang-format off
	EGLint image_attrs[] = {
		EGL_WIDTH, (EGLint)eglImage.width_,
		EGL_HEIGHT, (EGLint)eglImage.height_,
		EGL_LINUX_DRM_FOURCC_EXT, DRM_FORMAT_ARGB8888,
		EGL_DMA_BUF_PLANE0_FD_EXT, fd,
		EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
		EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)eglImage.stride_,
		EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, 0,
		EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, 0,
		EGL_NONE,
	};
	// clang-format on

	eglImage.image_ = eglCreateImageKHR(display_, EGL_NO_CONTEXT,
					    EGL_LINUX_DMA_BUF_EXT,
					    NULL, image_attrs);

	if (eglImage.image_ == EGL_NO_IMAGE_KHR) {
		LOG(eGL, Error) << "eglCreateImageKHR fail";
		ret = -ENODEV;
		goto done;
	}

	// Bind texture unit and texture
	glActiveTexture(eglImage.texture_unit_);
	glBindTexture(GL_TEXTURE_2D, eglImage.texture_);

	// Generate texture with filter semantics
	glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, eglImage.image_);

	// Nearest filtering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Wrap to edge to avoid edge artifacts
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	if (output) {
		// Generate a framebuffer from our texture direct to dma-buf handle buffer
		glBindFramebuffer(GL_FRAMEBUFFER, eglImage.fbo_);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, eglImage.texture_, 0);

		GLenum err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
		if (err != GL_FRAMEBUFFER_COMPLETE) {
			LOG(eGL, Error) << "glFrameBufferTexture2D error " << err;
			eglDestroyImageKHR(display_, eglImage.image_);
			ret = -ENODEV;
			goto done;
		}
	}
done:
	return ret;
}

/**
 * \brief Create an input DMA-BUF backed texture
 * \param[in,out] eglImage EGL image to associate with the DMA-BUF
 * \param[in] fd DMA-BUF file descriptor
 *
 * Creates an EGL image from a DMA-BUF file descriptor and binds it to
 * a 2D texture for use as an input texture in shaders. The texture is
 * configured with nearest filtering and clamp-to-edge wrapping.
 *
 * \return 0 on success, or -ENODEV on failure
 */
int eGL::createInputDMABufTexture2D(eGLImage &eglImage, int fd)
{
	ASSERT(tid_ == Thread::currentId());

	return createDMABufTexture2D(eglImage, fd, false);
}

/**
 * \brief Create an output DMA-BUF backed texture
 * \param[in,out] eglImage EGL image to associate with the DMA-BUF
 * \param[in] fd DMA-BUF file descriptor
 *
 * Creates an EGL image from a DMA-BUF file descriptor and binds it to
 * a 2D texture, then attaches it to a framebuffer object for use as a
 * render target. This enables zero-copy rendering directly to the
 * DMA-BUF.
 *
 * \return 0 on success, or -ENODEV on failure
 */
int eGL::createOutputDMABufTexture2D(eGLImage &eglImage, int fd)
{
	ASSERT(tid_ == Thread::currentId());

	return createDMABufTexture2D(eglImage, fd, true);
}

/**
 * \brief Destroy a DMA-BUF texture's EGL image
 * \param[in,out] eglImage EGL image to destroy
 *
 * Destroys the EGL image associated with a DMA-BUF texture. The OpenGL
 * texture and framebuffer objects are destroyed separately in the
 * eGLImage destructor.
 */
void eGL::destroyDMABufTexture(eGLImage &eglImage)
{
	eglDestroyImage(display_, std::exchange(eglImage.image_, EGL_NO_IMAGE_KHR));
}

/**
 * \brief Create a 2D texture from a memory buffer
 * \param[in,out] eglImage EGL image to associate with the texture
 * \param[in] format OpenGL internal format (e.g., GL_RGB, GL_RGBA)
 * \param[in] width Texture width in pixels
 * \param[in] height Texture height in pixels
 * \param[in] data Pointer to pixel data, or nullptr for uninitialised texture
 *
 * Creates a 2D texture from a CPU-accessible memory buffer. The texture
 * is configured with nearest filtering and clamp-to-edge wrapping. This
 * is useful for uploading static data like lookup tables or uniform color
 * matrices to the GPU.
 */
void eGL::createTexture2D(eGLImage &eglImage, GLint format, uint32_t width, uint32_t height, void *data)
{
	ASSERT(tid_ == Thread::currentId());

	glActiveTexture(eglImage.texture_unit_);
	glBindTexture(GL_TEXTURE_2D, eglImage.texture_);

	// Generate texture, bind, associate image to texture, configure, unbind
	glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);

	// Nearest filtering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Wrap to edge to avoid edge artifacts
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

/**
 * \brief Initialise the EGL context
 * \param[in] gbmContext Pointer to initialised GBM context
 *
 * Sets up the EGL display from the GBM device, creates an OpenGL ES 2.0
 * context, and retrieves function pointers for required extensions
 * including:
 * - eglCreateImageKHR / eglDestroyImageKHR
 * - glEGLImageTargetTexture2DOES
 *
 * \return 0 on success, or -ENODEV on failure
 */
int eGL::initEGLContext(GBM *gbmContext)
{
	EGLint configAttribs[] = {
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};

	EGLint contextAttribs[] = {
		EGL_CONTEXT_MAJOR_VERSION, 2,
		EGL_NONE
	};

	EGLint numConfigs;
	EGLConfig config;

	if (!eglBindAPI(EGL_OPENGL_ES_API)) {
		LOG(eGL, Error) << "API bind fail";
		goto fail;
	}

	display_ = eglGetDisplay(gbmContext->device());
	if (display_ == EGL_NO_DISPLAY) {
		LOG(eGL, Error) << "Unable to get EGL display";
		goto fail;
	}

	if (eglInitialize(display_, nullptr, nullptr) != EGL_TRUE) {
		LOG(eGL, Error) << "eglInitialize fail";
		goto fail;
	}

	LOG(eGL, Info) << "EGL: EGL_VERSION: " << eglQueryString(display_, EGL_VERSION);
	LOG(eGL, Info) << "EGL: EGL_VENDOR: " << eglQueryString(display_, EGL_VENDOR);
	LOG(eGL, Info) << "EGL: EGL_CLIENT_APIS: " << eglQueryString(display_, EGL_CLIENT_APIS);
	LOG(eGL, Info) << "EGL: EGL_EXTENSIONS: " << eglQueryString(display_, EGL_EXTENSIONS);

	eglCreateImageKHR = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
	if (!eglCreateImageKHR) {
		LOG(eGL, Error) << "eglCreateImageKHR not found";
		goto fail;
	}

	eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
	if (!eglDestroyImageKHR) {
		LOG(eGL, Error) << "eglDestroyImageKHR not found";
		goto fail;
	}

	glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
	if (!glEGLImageTargetTexture2DOES) {
		LOG(eGL, Error) << "glEGLImageTargetTexture2DOES not found";
		goto fail;
	}

	if (eglChooseConfig(display_, configAttribs, &config, 1, &numConfigs) != EGL_TRUE) {
		LOG(eGL, Error) << "eglChooseConfig fail";
		goto fail;
	}

	context_ = eglCreateContext(display_, config, EGL_NO_CONTEXT, contextAttribs);
	if (context_ == EGL_NO_CONTEXT) {
		LOG(eGL, Error) << "eglContext returned EGL_NO_CONTEXT";
		goto fail;
	}

	tid_ = Thread::currentId();

	makeCurrent();

	return 0;
fail:

	return -ENODEV;
}

/**
 * \brief Make the EGL context current for the calling thread
 *
 * Binds the EGL context to the current thread, allowing OpenGL ES
 * operations to be performed. Must be called from the thread that
 * will perform rendering operations.
 */
void eGL::makeCurrent()
{
	ASSERT(tid_ == Thread::currentId());

	if (eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_) != EGL_TRUE) {
		LOG(eGL, Error) << "eglMakeCurrent fail";
	}
}

/**
 * \brief Activate a shader program for rendering
 * \param[in] programId OpenGL program object ID
 *
 * Sets the specified program as the current rendering program. All
 * subsequent draw calls will use this program's shaders.
 */
void eGL::useProgram(GLuint programId)
{
	ASSERT(tid_ == Thread::currentId());

	glUseProgram(programId);
}

/**
 * \brief Delete a shader program
 * \param[in] programId OpenGL program object ID
 *
 * Deletes a shader program and frees associated resources. The program
 * must not be currently in use.
 */
void eGL::deleteProgram(GLuint programId)
{
	ASSERT(tid_ == Thread::currentId());

	glDeleteProgram(programId);
}

/**
 * \brief Add a preprocessor definition to shader environment
 * \param[in,out] shaderEnv Vector of shader environment strings
 * \param[in] str Preprocessor definition string (e.g., "#define APPLY_RGB_PARAMETERS")
 *
 * Appends a preprocessor definition to the shader environment vector.
 * These definitions are prepended to shader source code during compilation.
 */
void eGL::pushEnv(std::vector<std::string> &shaderEnv, const char *str)
{
	std::string addStr = str;

	addStr.push_back('\n');
	shaderEnv.push_back(std::move(addStr));
}

/**
 * \brief Compile a vertex shader
 * \param[out] shaderId OpenGL shader object ID
 * \param[in] shaderData Pointer to shader source code
 * \param[in] shaderDataLen Length of shader source in bytes
 * \param[in] shaderEnv Span of preprocessor definitions to prepend
 *
 * Compiles a vertex shader from source code with optional preprocessor
 * definitions. On compilation failure, logs the shader info log.
 *
 * \return 0 on success, or -EINVAL on compilation failure
 */
int eGL::compileVertexShader(GLuint &shaderId, const unsigned char *shaderData,
			     unsigned int shaderDataLen,
			     Span<const std::string> shaderEnv)
{
	return compileShader(GL_VERTEX_SHADER, shaderId, shaderData, shaderDataLen, shaderEnv);
}

/**
 * \brief Compile a fragment shader
 * \param[out] shaderId OpenGL shader object ID
 * \param[in] shaderData Pointer to shader source code
 * \param[in] shaderDataLen Length of shader source in bytes
 * \param[in] shaderEnv Span of preprocessor definitions to prepend
 *
 * Compiles a fragment shader from source code with optional preprocessor
 * definitions. On compilation failure, logs the shader info log.
 *
 * \return 0 on success, or -EINVAL on compilation failure
 */
int eGL::compileFragmentShader(GLuint &shaderId, const unsigned char *shaderData,
			       unsigned int shaderDataLen,
			       Span<const std::string> shaderEnv)
{
	return compileShader(GL_FRAGMENT_SHADER, shaderId, shaderData, shaderDataLen, shaderEnv);
}

/**
 * \brief Compile a shader of specified type
 * \param[in] shaderType GL_VERTEX_SHADER or GL_FRAGMENT_SHADER
 * \param[out] shaderId OpenGL shader object ID
 * \param[in] shaderData Pointer to shader source code
 * \param[in] shaderDataLen Length of shader source in bytes
 * \param[in] shaderEnv Span of preprocessor definitions to prepend
 *
 * Internal helper function for shader compilation. Prepends environment
 * definitions to the shader source and compiles the shader.
 *
 * \return 0 on success, or -EINVAL on compilation failure
 */
int eGL::compileShader(int shaderType, GLuint &shaderId, const unsigned char *shaderData,
		       unsigned int shaderDataLen,
		       Span<const std::string> shaderEnv)
{
	GLint success;
	size_t i;

	ASSERT(tid_ == Thread::currentId());

	auto count = 1 + shaderEnv.size();
	auto shaderSourceData = std::make_unique<const GLchar *[]>(count);
	auto shaderDataLengths = std::make_unique<GLint[]>(count);

	// Prefix defines before main body of shader
	for (i = 0; i < shaderEnv.size(); i++) {
		shaderSourceData[i] = shaderEnv[i].c_str();
		shaderDataLengths[i] = shaderEnv[i].length();
	}

	// Now the main body of the shader program
	shaderSourceData[i] = reinterpret_cast<const GLchar *>(shaderData);
	shaderDataLengths[i] = shaderDataLen;

	// And create the shader
	shaderId = glCreateShader(shaderType);
	glShaderSource(shaderId, count, shaderSourceData.get(), shaderDataLengths.get());
	glCompileShader(shaderId);

	// Check status
	glGetShaderiv(shaderId, GL_COMPILE_STATUS, &success);
	if (success == GL_FALSE) {
		GLint sizeLog = 0;

		glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &sizeLog);
		auto infoLog = std::make_unique<GLchar[]>(sizeLog);

		glGetShaderInfoLog(shaderId, sizeLog, &sizeLog, infoLog.get());
		LOG(eGL, Error) << infoLog.get();
	}

	return (success == GL_TRUE) ? 0 : -EINVAL;
}

/**
 * \brief Dump shader source code to the log
 * \param[in] shaderId OpenGL shader object ID
 *
 * Retrieves and logs the complete source code of a compiled shader.
 * Useful for debugging shader compilation issues.
 */
void eGL::dumpShaderSource(GLuint shaderId)
{
	GLint shaderLength = 0;

	ASSERT(tid_ == Thread::currentId());

	glGetShaderiv(shaderId, GL_SHADER_SOURCE_LENGTH, &shaderLength);

	LOG(eGL, Debug) << "Shader length is " << shaderLength;

	if (shaderLength > 0) {
		auto shaderSource = std::make_unique<GLchar[]>(shaderLength);

		glGetShaderSource(shaderId, shaderLength, &shaderLength, shaderSource.get());
		if (shaderLength) {
			LOG(eGL, Debug) << "Shader source = " << shaderSource.get();
		}
	}
}

/**
 * \brief Link a shader program
 * \param[out] programId OpenGL program object ID
 * \param[in] fragmentshaderId Compiled fragment shader ID
 * \param[in] vertexshaderId Compiled vertex shader ID
 *
 * Links vertex and fragment shaders into an executable shader program.
 * On link failure, logs the program info log and deletes the program.
 *
 * \return 0 on success, or -ENODEV on link failure
 */
int eGL::linkProgram(GLuint &programId, GLuint vertexshaderId, GLuint fragmentshaderId)
{
	GLint success;
	GLenum err;
	int ret = -ENODEV;

	ASSERT(tid_ == Thread::currentId());

	programId = glCreateProgram();
	if (!programId) {
		LOG(eGL, Error) << "glGreateProgram error";
		return ret;
	}

	utils::scope_exit programGuard([&] { glDeleteProgram(programId); });

	glAttachShader(programId, vertexshaderId);
	if ((err = glGetError()) != GL_NO_ERROR) {
		LOG(eGL, Error) << "Attach compute vertex shader fail err=" << err;
		return ret;
	}

	glAttachShader(programId, fragmentshaderId);
	if ((err = glGetError()) != GL_NO_ERROR) {
		LOG(eGL, Error) << "Attach compute fragment shader fail err=" << err;
		return ret;
	}

	glLinkProgram(programId);
	if ((err = glGetError()) != GL_NO_ERROR) {
		LOG(eGL, Error) << "Link program fail err=" << err;
		return ret;
	}

	glDetachShader(programId, fragmentshaderId);
	glDetachShader(programId, vertexshaderId);

	// Check status
	glGetProgramiv(programId, GL_LINK_STATUS, &success);
	if (success == GL_FALSE) {
		GLint sizeLog = 0;

		glGetProgramiv(programId, GL_INFO_LOG_LENGTH, &sizeLog);
		auto infoLog = std::make_unique<GLchar[]>(sizeLog);

		glGetProgramInfoLog(programId, sizeLog, &sizeLog, infoLog.get());
		LOG(eGL, Error) << infoLog.get();

		return ret;
	}

	programGuard.release();
	return 0;
}
} /* namespace libcamera */
