#!/usr/bin/python3

import pycamera as pycam

import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

from OpenGL import GL as gl
from OpenGL.EGL.KHR.image import *
from OpenGL.EGL.EXT.image_dma_buf_import import *
from OpenGL.EGL.VERSION.EGL_1_0 import *
from OpenGL.GLES2.VERSION.GLES2_2_0 import *

from OpenGL.GLES2.OES.EGL_image import *
from OpenGL.GLES2.OES.EGL_image_external import *

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QSize, Qt
from PyQt5.QtGui import (QColor, QSurfaceFormat)
from PyQt5.QtGui import (QOpenGLWindow, QOpenGLBuffer, QOpenGLShaderProgram, QOpenGLShader)
import sys
import numpy as np

import pykms

card = pykms.Card("/dev/dri/card0")
fb = pykms.DumbFramebuffer(card, 640, 480, "XR24");
pykms.draw_test_pattern(fb);

#fb = pykms.DmabufFramebuffer(card, origfb.width, origfb.height, origfb.format,
#		[origfb.fd(0)], [origfb.stride(0)], [origfb.offset(0)])


class MainWindow(QOpenGLWindow):
	def __init__(self):
		super().__init__()

		self.clearColor = QColor(Qt.red)

		#format = QtGui.QSurfaceFormat()
		#format.setDepthBufferSize(24)
		#format.setStencilBufferSize(8)
		#format.setVersion(2, 0)
		#format.setProfile(QtGui.QSurfaceFormat.CoreProfile)
		#format.setRenderableType(QtGui.QSurfaceFormat.OpenGLES)
		#self.setFormat(format)

	def sizeHint(self):
		return QSize(600, 600)

	def initializeGL(self):
		print("init")

		#gl.glEnable(gl.GL_DEPTH_TEST)
		#gl.glEnable(gl.GL_TEXTURE_2D)

		egl_display = EGLQuerier.getDisplay()

		print(eglQueryString(egl_display, EGL_VENDOR))

		extensions = eglQueryString(egl_display, EGL_EXTENSIONS).decode().split(" ")

		print("== EGL EXTENSIONS ==")
		print(extensions)

		if not "EGL_EXT_image_dma_buf_import" in extensions:
			raise Exception("EGL_EXT_image_dma_buf_import missing")

		assert(eglCreateImageKHR)
		assert(eglDestroyImageKHR)
		assert(glEGLImageTargetTexture2DOES)

		attribs = [
			EGL_WIDTH, fb.width,
			EGL_HEIGHT, fb.height,
			EGL_LINUX_DRM_FOURCC_EXT, int(fb.format),
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
								  #arrays.GLintArray.asArray(attribs))

		assert(image)

		textures = glGenTextures(1);
		glBindTexture(GL_TEXTURE_EXTERNAL_OES, textures);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, image);

		raise Exception("DONE")


		#pycam.hack_init_egl(self.winId())

		vertShaderSrc = """
			attribute vec4 pos;
			varying vec2 texcoord;

			void main() {
			  gl_Position = pos;
			  texcoord.x = pos.x;
			  texcoord.y = pos.y;
			}
			"""

		fragShaderSrc = """
			#extension GL_OES_EGL_image_external : enable
			precision mediump float;
			uniform samplerExternalOES s;
			varying vec2 texcoord;

			uniform sampler2D texture;
        	varying mediump vec4 texc;

			void main() {
				gl_FragColor = texture2D(texture, texcoord);
			}
			"""

		program = QOpenGLShaderProgram()
		program.addShaderFromSourceCode(QOpenGLShader.Vertex, vertShaderSrc)
		program.addShaderFromSourceCode(QOpenGLShader.Fragment, fragShaderSrc)
		program.link()
		program.bind()

		vertPositions = np.array([
			-0.5, -0.5,
			 0.5, -0.5,
			 0.5,  0.5,
			-0.5,  0.5], dtype=np.float32)
		vertPosBuffer = QOpenGLBuffer()
		vertPosBuffer.create()
		vertPosBuffer.bind()
		vertPosBuffer.allocate(vertPositions, len(vertPositions) * 4)

		self.vertPosBuffer = vertPosBuffer


		img = QtGui.QImage('/home/tomba/Downloads/Kovat materiaalit/Viilu/Veneers/Koivu.jpg')
		txt = QtGui.QOpenGLTexture(img)
		self.texture = txt

		program.setAttributeBuffer(0, gl.GL_FLOAT, 0, 2)
		program.enableAttributeArray(0)

		#program.setUniformValue("texture", 0);

		#gl.glBindTexture(GL_TEXTURE_EXTERNAL_OES, 0);

		#gl.glTexParameteri(gl.GL_TEXTURE_EXTERNAL_OES, gl.GL_TEXTURE_MIN_FILTER, gl.GL_LINEAR)
		#gl.glTexParameteri(gl.GL_TEXTURE_EXTERNAL_OES, gl.GL_TEXTURE_MAG_FILTER, gl.GL_LINEAR)


		#gl.glEGLImageTargetTexture2DOES(gl.GL_TEXTURE_EXTERNAL_OES, 0);



		#v = pycam.hack_create_buffer(origfb.fd(0), origfb.width, origfb.height, origfb.stride(0))
		#print("CREATE", v)


#	glGenTextures(1, &buf_kmsc->texture_name);
#	glBindTexture(GL_TEXTURE_EXTERNAL_OES, buf_kmsc->texture_name);
#
#	if (glGetError() != GL_NO_ERROR) {
#		ERROR("glBindTexture!");
#		goto fail;
#	}
#
#        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
#        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
#
#	// Update video texture / EGL Image.
#        disp_kmsc->gl.glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, buf_kmsc->egl_img);



	def paintGL(self):
		print("paint")
		return

		c = self.clearColor

		gl.glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

		gl.glClear(gl.GL_COLOR_BUFFER_BIT)

		self.texture.bind()
		gl.glDrawArrays(gl.GL_TRIANGLE_FAN, 0, 4);

	def resizeGL(self, w, h):
		print("resize", w, h)
		#self.gl.glViewport(0, 0, w, h)


app = QtWidgets.QApplication([])

window = MainWindow()
window.show()

app.exec()

