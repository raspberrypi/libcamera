#!/usr/bin/python3

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QSize, Qt
from PyQt5.QtCore import QTimer

import numpy as np

import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

from ctypes import c_int, c_char_p, c_void_p, cdll, POINTER, util, \
	pointer, CFUNCTYPE, c_bool

import OpenGL
#OpenGL.FULL_LOGGING = True

from OpenGL import GL as gl
from OpenGL.EGL.KHR.image import *
from OpenGL.EGL.EXT.image_dma_buf_import import *
from OpenGL.EGL.VERSION.EGL_1_0 import *
from OpenGL.EGL.VERSION.EGL_1_2 import *
from OpenGL.EGL.VERSION.EGL_1_3 import *

from OpenGL.GLES2.VERSION.GLES2_2_0 import *
from OpenGL.GLES2.OES.EGL_image import *
from OpenGL.GLES2.OES.EGL_image_external import *
from OpenGL.GLES3.VERSION.GLES3_3_0 import *

from OpenGL.GL import shaders

import pykms

card = pykms.Card("/dev/dri/card1")
print("DRM CARD", card.version_name)
fb = pykms.DumbFramebuffer(card, 640, 480, "XR24");
pykms.draw_test_pattern(fb);


_x11lib = cdll.LoadLibrary(util.find_library("X11"))
XOpenDisplay = _x11lib.XOpenDisplay
XOpenDisplay.argtypes = [c_char_p]
XOpenDisplay.restype = POINTER(EGLNativeDisplayType)

xdpy = XOpenDisplay(None)

egl_display = eglGetDisplay(xdpy)

major, minor = EGLint(), EGLint()

b = eglInitialize(egl_display, major, minor)
assert(b)

verbose = True

if verbose:
	print("eglInitialize: {}.{}".format(major.value, minor.value))
	print("EGL_VENDOR:      {}".format(eglQueryString(egl_display, EGL_VENDOR).decode()))
	print("EGL_VERSION:     {}".format(eglQueryString(egl_display, EGL_VERSION).decode()))
	print("EGL_EXTENSIONS:  {}".format(eglQueryString(egl_display, EGL_EXTENSIONS).decode()))
	print("EGL_CLIENT_APIS: {}".format(eglQueryString(egl_display, EGL_CLIENT_APIS).decode()))

extensions = eglQueryString(egl_display, EGL_EXTENSIONS).decode().split(" ")

if not "EGL_EXT_image_dma_buf_import" in extensions:
	raise Exception("EGL_EXT_image_dma_buf_import missing")

b = eglBindAPI(EGL_OPENGL_ES_API)
assert(b)

def print_config(dpy, cfg):

	def _getconf(dpy, cfg, a):
		value = ctypes.c_long()
		eglGetConfigAttrib(dpy, cfg, a, value)
		return value.value

	getconf = lambda a: _getconf(dpy, cfg, a)

	print("EGL Config {}: color buf {}/{}/{}/{} = {}, depth {}, stencil {}, native visualid {}, native visualtype {}".format(
		getconf(EGL_CONFIG_ID),
		getconf(EGL_ALPHA_SIZE),
		getconf(EGL_RED_SIZE),
		getconf(EGL_GREEN_SIZE),
		getconf(EGL_BLUE_SIZE),
		getconf(EGL_BUFFER_SIZE),
		getconf(EGL_DEPTH_SIZE),
		getconf(EGL_STENCIL_SIZE),
		getconf(EGL_NATIVE_VISUAL_ID),
		getconf(EGL_NATIVE_VISUAL_TYPE)))


if False:
	num_configs = ctypes.c_long()
	eglGetConfigs(egl_display, None, 0, num_configs)
	print("{} configs".format(num_configs.value))

	configs = (EGLConfig * num_configs.value)()
	eglGetConfigs(egl_display, configs, num_configs.value, num_configs)
	for config_id in configs:
		print_config(egl_display, config_id)


config_attribs = [
	EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
	EGL_RED_SIZE, 8,
	EGL_GREEN_SIZE, 8,
	EGL_BLUE_SIZE, 8,
	EGL_ALPHA_SIZE, 0,
	EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
	EGL_NONE,
]

n = EGLint()
configs = (EGLConfig * 1)()
b = eglChooseConfig(egl_display, config_attribs, configs, 1, n)
assert(b and n.value == 1)
config = configs[0]

print("Chosen Config:")
print_config(egl_display, config)

context_attribs = [
	EGL_CONTEXT_CLIENT_VERSION, 2,
	EGL_NONE,
]

context = eglCreateContext(egl_display, config, EGL_NO_CONTEXT, context_attribs)
assert(context)

b = eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, context)
assert(b)




n = GLint()
gl.glGetIntegerv(GL_NUM_EXTENSIONS, n)
gl_extensions = []
for i in range(n.value):
	gl_extensions.append(gl.glGetStringi(GL_EXTENSIONS, i).decode())

if False:
	print("GL_EXTENSIONS: ", " ".join(gl_extensions))

if not "GL_OES_EGL_image" in gl_extensions:
	raise Exception("GL_OES_EGL_image missing")



from ctypes import CFUNCTYPE
from OpenGL.raw.GLES2 import _types as _cs

funcptr = eglGetProcAddress("glEGLImageTargetTexture2DOES")

prototype = CFUNCTYPE(None,_cs.GLenum,_cs.GLeglImageOES)
glEGLImageTargetTexture2DOES = prototype(funcptr)

#a = glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, None)
#print(a)



assert(eglCreateImageKHR)
assert(eglDestroyImageKHR)
assert(glEGLImageTargetTexture2DOES)



class MainWindow(QtWidgets.QWidget):
	def __init__(self):
		super().__init__()

		self.setAttribute(Qt.WA_PaintOnScreen)
		self.setAttribute(Qt.WA_NativeWindow)

		self.surface = None

	def paintEngine(self):
		return None

	def init_gl(self):
		surface = eglCreateWindowSurface(egl_display, config, c_void_p(window.winId().__int__()), None)

		b = eglMakeCurrent(egl_display, surface, surface, context)
		assert(b)

		self.surface = surface
		self.egl_display = egl_display


		gl.glClearColor(0.5, 0.8, 0.7, 1.0)

		vertShaderSrc = """
			attribute vec2 aPosition;
			varying vec2 texcoord;

			void main()
			{
				gl_Position = vec4(aPosition * 2.0 - 1.0, 0.0, 1.0);
				texcoord.x = aPosition.x;
				texcoord.y = aPosition.y;
			}
		"""
		fragShaderSrc = """
			#extension GL_OES_EGL_image_external : enable
			precision mediump float;
			varying vec2 texcoord;
			uniform samplerExternalOES texture;

			void main()
			{
				gl_FragColor = texture2D(texture, texcoord);
			}
		"""

		program = shaders.compileProgram(
		    shaders.compileShader(vertShaderSrc, GL_VERTEX_SHADER),
		    shaders.compileShader(fragShaderSrc, GL_FRAGMENT_SHADER)
		)

		for i in range(glGetProgramiv(program, GL_ACTIVE_ATTRIBUTES)):
			name,size,type = glGetActiveAttrib( program, i )
			print( 'Arribute %s(%i) -> %s %s'%(name,i,size,type))

		for i in range(glGetProgramiv(program, GL_ACTIVE_UNIFORMS)):
			name,size,type = glGetActiveUniform( program, i )
			print( 'Uniform %s(%i) -> %s %s'%(name,i,size,type))

		#glBindAttribLocation(program, 0, "aPosition")

		#glLinkProgram(program)
		glUseProgram(program)

		#program.link()
		#program.bind()

		vertPositions = (GLfloat * 9)(
			 0.0,  0.0,
			 1.0,  0.0,
			 1.0,  1.0,
			 0.0,  1.0)

		#vao = glGenVertexArrays(1)
		#glBindVertexArray(vao)

		#vbo = glGenBuffers(1)
		#glBindBuffer(GL_ARRAY_BUFFER, vbo)
		#glBufferData(GL_ARRAY_BUFFER, ctypes.sizeof(vertPositions), vertPositions, GL_STATIC_DRAW)

		inputAttrib = glGetAttribLocation(program, "aPosition")
		glVertexAttribPointer(inputAttrib, 2, GL_FLOAT, GL_FALSE, 0, vertPositions)
		glEnableVertexAttribArray(inputAttrib)



		if False:
			w = 640
			h = 480
			data = np.random.randint(0, 255, w*h*4).astype(np.ubyte)

			glActiveTexture(GL_TEXTURE0)

			textures = glGenTextures(1)
			glBindTexture(GL_TEXTURE_2D, textures)
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_RGB, GL_UNSIGNED_BYTE, data)
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)

		if True:
			attribs = [
				EGL_WIDTH, fb.width,
				EGL_HEIGHT, fb.height,
				EGL_LINUX_DRM_FOURCC_EXT, fb.format,
				EGL_DMA_BUF_PLANE0_FD_EXT, fb.fd(0),
				EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
				EGL_DMA_BUF_PLANE0_PITCH_EXT, fb.stride(0),
				EGL_NONE,
			]

			image = eglCreateImageKHR(egl_display,
			                          EGL_NO_CONTEXT,
									  EGL_LINUX_DMA_BUF_EXT,
									  None,
									  attribs)
			assert(image)

			#glActiveTexture(GL_TEXTURE0)

			textures = glGenTextures(1)
			glBindTexture(GL_TEXTURE_EXTERNAL_OES, textures)
			glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
			glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
			glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
			glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
			glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, image);

			self.textures = textures

	def resizeEvent(self, event):
		size = event.size()

		print("Resize", size)

		super().resizeEvent(event)

		if self.surface == None:
			return

		glViewport(0, 0, size.width(), size.height())

	def paintEvent(self, event):
		print("paint")
		self.paint_gl()

	def paint_gl(self):
		glClear(GL_COLOR_BUFFER_BIT)

		glBindTexture(GL_TEXTURE_EXTERNAL_OES, self.textures);
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

		b = eglSwapBuffers(self.egl_display, self.surface)
		assert(b)




app = QtWidgets.QApplication([])
window = MainWindow()
window.resize(1000, 1000)
window.show()

window.init_gl()

def tick():
	window.update()
	#window.repaint()
#	window.paint_gl()

timer = QTimer()
timer.timeout.connect(tick)
timer.start(500)

app.exec()
