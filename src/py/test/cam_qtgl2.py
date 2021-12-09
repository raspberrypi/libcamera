from PyQt5 import QtCore, QtWidgets, QtGui
#from PyQt5.QtGui import QWindow
from PyQt5.QtCore import Qt
from PyQt5.QtCore import Qt, QEvent

import numpy as np
import sys
import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

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

from gl_helpers import *

FMT_MAP = {
	"RGB888": "RG24",
	"XRGB8888": "XR24",
	"ARGB8888": "AR24",
	"YUYV": "YUYV",
}

class QtRenderer:
	def __init__(self, state):
		self.state = state

		self.cm = state["cm"]
		self.contexts = state["contexts"]

	def setup(self):
		self.egl_setup()

		self.app = QtWidgets.QApplication([])

		windows = []

		for ctx in self.contexts:
			camera = ctx["camera"]

			# for more than 1 stream, the window has to be changed to render two streams
			assert(len(ctx["streams"]) == 1)

			stream = next(iter(ctx["streams"]))
			fmt = stream.configuration.fmt
			size = stream.configuration.size

			if not fmt in FMT_MAP:
				raise Exception("Unsupported pixel format")

			window = MainWindow(self.state, ctx, stream, self.egl_display, self.config, self.egl_context)
			#window.setAttribute(QtCore.Qt.WA_ShowWithoutActivating)
			window.resize(600, 600)
			window.show()
			window.init_gl()
			windows.append(window)

		self.windows = windows

	def egl_setup(self):
		xdpy = getEGLNativeDisplay()
		egl_display = eglGetDisplay(xdpy)
		self.egl_display = egl_display

		major, minor = EGLint(), EGLint()

		b = eglInitialize(egl_display, major, minor)
		assert(b)

		print("EGL {} {}".format(
		      eglQueryString(egl_display, EGL_VENDOR).decode(),
		      eglQueryString(egl_display, EGL_VERSION).decode()))

		check_egl_extensions(egl_display, ["EGL_EXT_image_dma_buf_import"])

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

		self.config = config

		context_attribs = [
			EGL_CONTEXT_CLIENT_VERSION, 2,
			EGL_NONE,
		]

		context = eglCreateContext(egl_display, config, EGL_NO_CONTEXT, context_attribs)
		assert(context)

		self.egl_context = context

		b = eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, context)
		assert(b)


		check_gl_extensions(["GL_OES_EGL_image"])

		assert(eglCreateImageKHR)
		assert(eglDestroyImageKHR)
		assert(glEGLImageTargetTexture2DOES)



	def run(self):
		camnotif = QtCore.QSocketNotifier(self.cm.efd, QtCore.QSocketNotifier.Read)
		camnotif.activated.connect(lambda x: self.readcam())

		keynotif = QtCore.QSocketNotifier(sys.stdin.fileno(), QtCore.QSocketNotifier.Read)
		keynotif.activated.connect(lambda x: self.readkey())

		print("Capturing...")

		self.app.exec()

		print("Exiting...")

	def readcam(self):
		running = self.state["event_handler"](self.state)

		if not running:
			self.app.quit()

	def readkey(self):
		sys.stdin.readline()
		self.app.quit()

	def request_handler(self, ctx, req):

		wnd = next(wnd for wnd in self.windows if wnd.ctx == ctx)

		wnd.handle_request(req)

	def cleanup(self):
		for w in self.windows:
			w.close()


class MainWindow(QtGui.QWindow):
	def __init__(self, state, ctx, stream, egl_display, egl_config, egl_context):
		super().__init__()

		self.state = state
		self.ctx = ctx
		self.stream = stream
		self.egl_display = egl_display
		self.egl_config = egl_config
		self.egl_context = egl_context

		self.setSurfaceType(QtGui.QWindow.OpenGLSurface);

		#self.setAttribute(Qt.WA_PaintOnScreen)
		#self.setAttribute(Qt.WA_NativeWindow)

		self.surface = None
		self.textures = None

		self.reqqueue = []
		self.next = None


	def paintEngine(self):
		return None

	def init_gl(self):

		vertShaderSrc = """
			attribute vec2 aPosition;
			varying vec2 texcoord;

			void main()
			{
				gl_Position = vec4(aPosition * 2.0 - 1.0, 0.0, 1.0);
				texcoord.x = aPosition.x;
				texcoord.y = 1.0 - aPosition.y;
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


		native_surface = c_void_p(self.winId().__int__())
		surface = eglCreateWindowSurface(self.egl_display, self.egl_config,
		                                 native_surface, None)

		b = eglMakeCurrent(self.egl_display, self.surface, self.surface, self.egl_context)
		assert(b)

		self.surface = surface

		glClearColor(0.5, 0.8, 0.7, 1.0)


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


	def create_text(self, stream, fb):
		cfg = stream.configuration
		fmt = cfg.fmt
		fmt = str_to_fourcc(FMT_MAP[fmt])
		w, h = cfg.size

		attribs = [
			EGL_WIDTH, w,
			EGL_HEIGHT, h,
			EGL_LINUX_DRM_FOURCC_EXT, fmt,
			EGL_DMA_BUF_PLANE0_FD_EXT, fb.fd(0),
			EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
			EGL_DMA_BUF_PLANE0_PITCH_EXT, cfg.stride,
			EGL_NONE,
		]

		image = eglCreateImageKHR(self.egl_display,
		                          EGL_NO_CONTEXT,
								  EGL_LINUX_DMA_BUF_EXT,
								  None,
								  attribs)
		assert(image)

		textures = glGenTextures(1)
		assert(textures)

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

	def paint_gl(self):
		b = eglMakeCurrent(self.egl_display, self.surface, self.surface, self.egl_context)
		assert(b)

		glClear(GL_COLOR_BUFFER_BIT)

		glBindTexture(GL_TEXTURE_EXTERNAL_OES, self.textures);
		glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

		b = eglSwapBuffers(self.egl_display, self.surface)
		assert(b)

	def handle_request(self, req):

		assert(len(req.buffers) == 1)

		self.reqqueue.append(req)
		#self.update()
		print("REQ UPDATE")
		self.requestUpdate()

	def event(self, event):
		if event.type() == QEvent.UpdateRequest:
			print("UpdateRequest")
			self.renderNow()
			return True
		else:
			return super().event(event)

	def renderNow(self):
		print("render now")
		if not self.isExposed():
				return

		if len(self.reqqueue) == 0:
			return

		if self.next:
			old = self.next
			self.next = None
			self.state["request_prcessed"](self.ctx, old)

		self.next = self.reqqueue.pop(0)

		req = self.next

		stream, fb = next(iter(req.buffers.items()))

		self.create_text(stream, fb)

		self.paint_gl()

	def exposeEvent(self, event):
		print("exposeEvent")

		if self.isExposed():
			self.renderNow()
