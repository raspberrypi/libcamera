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
#include <linux/drm_fourcc.h>

#include <libcamera/base/thread.h>

#include <GLES3/gl32.h>

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
 * \brief Construct an EGL helper
 * \param[in] display The EGL display to use
 *
 * Creates an eGL instance with uninitialised context. Call initEGLContext()
 * to set up the EGL context, and load extension functions.
 */
eGL::eGL(EGLDisplay display)
	: display_(display)
{
	ASSERT(display_ != EGL_NO_DISPLAY);
}

/**
 * \brief Destroy the EGL helper
 *
 * An instance must only be destroyed if there is no active EGL context.
 *
 * \sa resetEGLContext()
 */
eGL::~eGL()
{
	ASSERT(context_ == EGL_NO_CONTEXT);
}

/**
 * \brief Synchronise rendering output
 *
 * Sychronise here. Calls glFinish() right now.
 *
 */
void eGL::syncOutput()
{
	assertThread();

	glFinish();
}

/**
 * \brief Flush the rendering pipeline
 *
 * Calls glFlush().
 *
 */
void eGL::flushOutput()
{
	assertThread();

	glFlush();
}

/**
 * \fn eGL::assertThread()
 * \brief Assert that the execution is in the right thread
 *
 * Asserts that the execution is in the thread for which the created EGL
 * context has been made current.
 */

/**
 * \brief Attach a texture to a frame-buffer-object
 *
 * \param[in,out] eglImage EGL image containing texture to attach to FBO
 *
 * Helper function to make attachment of texture to FBO easy to reuse.
 *
 * \return 0 on success, or -ENODEV on failure
 */
int eGL::attachTextureToFBO(eGLImage &eglImage)
{
	int ret = 0;

	// Generate a framebuffer from our texture direct to dma-buf handle buffer
	glBindFramebuffer(GL_FRAMEBUFFER, eglImage.fbo_);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, eglImage.texture_, 0);

	GLenum err = glCheckFramebufferStatus(GL_FRAMEBUFFER);
	if (err != GL_FRAMEBUFFER_COMPLETE) {
		LOG(eGL, Error) << "glFrameBufferTexture2D error " << err;
		ret = -ENODEV;
	}

	return ret;
}

/**
 * \brief Activate a texture unit and bind a texture to that unit
 * \param[in,out] eglImage EGL image containing data related to unit and texture id
 *
 * When we create a texture we will bind a texture unit and texture id so
 * we can set filters. For the case where a texture already exists though
 * we need to activate and bind an existing texture. This helper function
 * facilitates both cases.
 *
 */
void eGL::activateBindTexture(eGLImage &eglImage)
{
	// Bind texture unit and texture
	glActiveTexture(eglImage.texture_unit_);
	glBindTexture(GL_TEXTURE_2D, eglImage.texture_);
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
	EGLint drm_format;
	int ret = 0;

	assertThread();

	switch (eglImage.format_) {
	case GL_RED:
	case GL_LUMINANCE:
		drm_format = DRM_FORMAT_R8;
		break;
	case GL_RG:
		drm_format = DRM_FORMAT_RG88;
		break;
	case GL_RGBA:
		drm_format = DRM_FORMAT_ARGB8888;
		break;
	default:
		LOG(eGL, Error) << "unhandled GL format";
		return -ENODEV;
	}

	// clang-format off
	EGLint image_attrs[] = {
		EGL_WIDTH, (EGLint)eglImage.width_,
		EGL_HEIGHT, (EGLint)eglImage.height_,
		EGL_LINUX_DRM_FOURCC_EXT, drm_format,
		EGL_DMA_BUF_PLANE0_FD_EXT, fd,
		EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
		EGL_DMA_BUF_PLANE0_PITCH_EXT, (EGLint)eglImage.stride_,
		EGL_DMA_BUF_PLANE0_MODIFIER_LO_EXT, 0,
		EGL_DMA_BUF_PLANE0_MODIFIER_HI_EXT, 0,
		EGL_NONE,
	};
	// clang-format on

	EGLImageKHR image = vtable_.eglCreateImageKHR(display_, EGL_NO_CONTEXT,
						      EGL_LINUX_DMA_BUF_EXT,
						      NULL, image_attrs);

	if (image == EGL_NO_IMAGE_KHR) {
		LOG(eGL, Debug) << "eglCreateImageKHR fail";
		eglImage.dmabuf_import_failed_ = true;
		return -ENODEV;
	}

	activateBindTexture(eglImage);

	// Generate texture with filter semantics
	vtable_.glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, image);
	vtable_.eglDestroyImageKHR(display_, image);

	// Nearest filtering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Wrap to edge to avoid edge artifacts
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	if (output)
		ret = attachTextureToFBO(eglImage);

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
	assertThread();

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
	assertThread();

	return createDMABufTexture2D(eglImage, fd, true);
}

/**
 * \brief Create a 2D texture from a memory buffer
 * \param[in,out] eglImage EGL image to associate with the texture
 * \param[in] data Pointer to pixel data, or nullptr for uninitialised texture
 *
 * Creates a 2D texture from a CPU-accessible memory buffer. The texture
 * is configured with nearest filtering and clamp-to-edge wrapping. This
 * is useful for uploading static data like lookup tables or uniform color
 * matrices to the GPU.
 */
void eGL::createTexture2D(eGLImage &eglImage, void *data)
{
	assertThread();

	activateBindTexture(eglImage);

	// Generate texture, bind, associate image to texture, configure, unbind
	glTexImage2D(GL_TEXTURE_2D, 0, eglImage.format_, eglImage.width_, eglImage.height_, 0, eglImage.format_, GL_UNSIGNED_BYTE, data);

	// Nearest filtering
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	// Wrap to edge to avoid edge artifacts
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

/**
 * \brief Try to create an EGL display for surfaceless rendering
 *
 * Tries to create an EGL display for surfaceless rendering.
 *
 * \return A EGL display handle if successful, otherwise \a EGL_NO_DISPLAY
 */
EGLDisplay eGL::probeDisplay()
{
	EGLDisplay display;

	display = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
					EGL_DEFAULT_DISPLAY,
					nullptr);

	if (display == EGL_NO_DISPLAY) {
		LOG(eGL, Info) << "Unable to get EGL display";
		return EGL_NO_DISPLAY;
	}

	if (eglInitialize(display, nullptr, nullptr) != EGL_TRUE) {
		LOG(eGL, Error) << "eglInitialize fail";
		return EGL_NO_DISPLAY;
	}

	return display;
}

/**
 * \brief Update a 2D texture already created
 * \param[in,out] eglImage EGL image to associate with the texture
 * \param[data] Data to update the texture with
 *
 * Updates a 2D texture in VRAM.
 */
void eGL::updateTexture2D(eGLImage &eglImage, void *data)
{
	assertThread();

	activateBindTexture(eglImage);

	// Update an already existing texture
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, eglImage.width_, eglImage.height_, eglImage.format_, GL_UNSIGNED_BYTE, data);
}

/**
 * \brief Create a 2D texture attached to an FBO for render-to-texture
 * \param[in,out] eglImage EGL image to associate with the texture
 *
 * Creates a 2D texture in VRAM. Subsequetly attach the texture to the
 * texture unit specified in the eGLImage object.
 */
void eGL::createOutputTexture2D(eGLImage &eglImage)
{
	createTexture2D(eglImage, NULL);
	attachTextureToFBO(eglImage);
}

/**
 * \brief Initialise the EGL context
 *
 * Sets up the EGL display, creates an OpenGL ES 2.0 context, and retrieves
 * function pointers for required extensions including:
 * - eglCreateImageKHR / eglDestroyImageKHR
 * - glEGLImageTargetTexture2DOES
 *
 * Each successful invocation must be followed by a call to resetEGLContext().
 *
 * If a valid EGL context is already present, initialization attempts
 * will be rejected.
 *
 * \return 0 on success, or negative error code on failure
 */
int eGL::initEGLContext()
{
	EGLint configAttribs[] = {
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 8,
		EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};

	EGLint contextAttribs[] = {
		EGL_CONTEXT_MAJOR_VERSION, 2,
		EGL_NONE
	};

	VTable vtable = {};
	EGLint numConfigs;
	EGLConfig config;

	if (context_ != EGL_NO_CONTEXT)
		return -EEXIST;

	if (!eglBindAPI(EGL_OPENGL_ES_API)) {
		LOG(eGL, Error) << "API bind fail";
		goto fail;
	}

	LOG(eGL, Info) << "EGL: EGL_VERSION: " << eglQueryString(display_, EGL_VERSION);
	LOG(eGL, Info) << "EGL: EGL_VENDOR: " << eglQueryString(display_, EGL_VENDOR);
	LOG(eGL, Info) << "EGL: EGL_CLIENT_APIS: " << eglQueryString(display_, EGL_CLIENT_APIS);
	LOG(eGL, Info) << "EGL: EGL_EXTENSIONS: " << eglQueryString(display_, EGL_EXTENSIONS);

	vtable.eglCreateImageKHR = (PFNEGLCREATEIMAGEKHRPROC)eglGetProcAddress("eglCreateImageKHR");
	if (!vtable.eglCreateImageKHR) {
		LOG(eGL, Error) << "eglCreateImageKHR not found";
		goto fail;
	}

	vtable.eglDestroyImageKHR = (PFNEGLDESTROYIMAGEKHRPROC)eglGetProcAddress("eglDestroyImageKHR");
	if (!vtable.eglDestroyImageKHR) {
		LOG(eGL, Error) << "eglDestroyImageKHR not found";
		goto fail;
	}

	vtable.glEGLImageTargetTexture2DOES = (PFNGLEGLIMAGETARGETTEXTURE2DOESPROC)eglGetProcAddress("glEGLImageTargetTexture2DOES");
	if (!vtable.glEGLImageTargetTexture2DOES) {
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
	vtable_ = vtable;

	makeCurrent();

	LOG(eGL, Info) << "EGL: GL_RENDERER: " << glGetString(GL_RENDERER);
	LOG(eGL, Info) << "EGL: GL_VERSION: " << glGetString(GL_VERSION);

	return 0;
fail:

	return -ENODEV;
}

/**
 * \brief Destroy the EGL context
 *
 * This function destroys the EGL context created by initEGLContext().
 * If there is no valid EGL context, this function has no effect.
 */
void eGL::resetEGLContext()
{
	if (context_ != EGL_NO_CONTEXT) {
		assertThread();
		eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
		eglDestroyContext(display_, std::exchange(context_, EGL_NO_CONTEXT));
	}

	tid_ = -1;
	vtable_ = {};
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
	assertThread();

	if (eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, context_) != EGL_TRUE) {
		LOG(eGL, Error) << "eglMakeCurrent fail";
	}
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
 * \param[in] shaderData Shader source code
 * \param[in] shaderEnv Span of preprocessor definitions to prepend
 *
 * Compiles a vertex shader from source code with optional preprocessor
 * definitions. On compilation failure, logs the shader info log.
 *
 * \return 0 on success, or -EINVAL on compilation failure
 */
int eGL::compileVertexShader(GLuint &shaderId,
			     std::span<const unsigned char> shaderData,
			     std::span<const std::string> shaderEnv)
{
	return compileShader(GL_VERTEX_SHADER, shaderId, shaderData, shaderEnv);
}

/**
 * \brief Compile a fragment shader
 * \param[out] shaderId OpenGL shader object ID
 * \param[in] shaderData Shader source code
 * \param[in] shaderEnv Span of preprocessor definitions to prepend
 *
 * Compiles a fragment shader from source code with optional preprocessor
 * definitions. On compilation failure, logs the shader info log.
 *
 * \return 0 on success, or -EINVAL on compilation failure
 */
int eGL::compileFragmentShader(GLuint &shaderId,
			       std::span<const unsigned char> shaderData,
			       std::span<const std::string> shaderEnv)
{
	return compileShader(GL_FRAGMENT_SHADER, shaderId, shaderData, shaderEnv);
}

/**
 * \brief Compile a shader of specified type
 * \param[in] shaderType GL_VERTEX_SHADER or GL_FRAGMENT_SHADER
 * \param[out] shaderId OpenGL shader object ID
 * \param[in] shaderData Shader source code
 * \param[in] shaderEnv Span of preprocessor definitions to prepend
 *
 * Internal helper function for shader compilation. Prepends environment
 * definitions to the shader source and compiles the shader.
 *
 * \return 0 on success, or -EINVAL on compilation failure
 */
int eGL::compileShader(int shaderType, GLuint &shaderId,
		       std::span<const unsigned char> shaderData,
		       std::span<const std::string> shaderEnv)
{
	GLint success;
	size_t i;

	assertThread();

	auto count = 1 + shaderEnv.size();
	auto shaderSourceData = std::make_unique<const GLchar *[]>(count);
	auto shaderDataLengths = std::make_unique<GLint[]>(count);

	// Prefix defines before main body of shader
	for (i = 0; i < shaderEnv.size(); i++) {
		shaderSourceData[i] = shaderEnv[i].c_str();
		shaderDataLengths[i] = shaderEnv[i].length();
	}

	// Now the main body of the shader program
	shaderSourceData[i] = reinterpret_cast<const GLchar *>(shaderData.data());
	shaderDataLengths[i] = shaderData.size();

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

	assertThread();

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

	assertThread();

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
