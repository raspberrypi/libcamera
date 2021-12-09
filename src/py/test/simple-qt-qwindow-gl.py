#!/usr/bin/python3

import sys

import os
os.environ["PYOPENGL_PLATFORM"] = "egl"

import numpy as np

from OpenGL import GL as gl
from OpenGL.EGL.KHR.image import *

from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QWindow
from PyQt5.QtCore import Qt, QEvent

class OpenGLWindow(QWindow):
	def __init__(self):
		super().__init__()
		self.setSurfaceType(QWindow.OpenGLSurface);
		self.setTitle("Triangle, PyQt5, OpenGL ES 2.0")
		self.resize(400, 400)

	def initialize(self):
		print("init")

	def render(self):
		print("render")

	def renderLater(self):
		print("renderlater")
		self.requestUpdate()

	def event(self, event):
		if event.type() == QEvent.UpdateRequest:
			print("UpdateRequest")
			self.renderNow()
			return True
		else:
			return super().event(event)

	def exposeEvent(self, event):
		print("exposeEvent")

		if self.isExposed():
			self.renderNow()

	def renderNow(self):
		print("render now")

		if not self.isExposed():
			return

		needInit = False

		if self.context == None:
			# create context
			needInit = True

		# make current

		if needInit:
			#call own init func
			pass

		self.render()

		# swap buf


	def initializeGL(self):

		assert(eglCreateImageKHR)

		gl.glClearColor(0.5, 0.8, 0.7, 1.0)
		vertShaderSrc = """
			attribute vec3 aPosition;
			void main()
			{
				gl_Position = vec4(aPosition, 1.0);
			}
		"""
		fragShaderSrc = """
			void main()
			{
				gl_FragColor = vec4(0.5, 0.2, 0.9, 1.0);
			}
		"""
		program = QOpenGLShaderProgram(self)
		program.addShaderFromSourceCode(QOpenGLShader.Vertex, vertShaderSrc)
		program.addShaderFromSourceCode(QOpenGLShader.Fragment, fragShaderSrc)
		program.link()
		program.bind()
		vertPositions = np.array([
			-0.5, -0.5, 0.0,
			0.5, -0.5, 0.0,
			0.0, 0.5, 0.0], dtype=np.float32)
		self.vertPosBuffer = QOpenGLBuffer()
		self.vertPosBuffer.create()
		self.vertPosBuffer.bind()
		self.vertPosBuffer.allocate(vertPositions, len(vertPositions) * 4)
		program.bindAttributeLocation("aPosition", 0)
		program.setAttributeBuffer(0, gl.GL_FLOAT, 0, 3)
		program.enableAttributeArray(0)

	def paintGL(self):
		gl.glClear(gl.GL_COLOR_BUFFER_BIT)
		gl.glDrawArrays(gl.GL_TRIANGLES, 0, 3)

def main():
	QApplication.setAttribute(Qt.AA_UseDesktopOpenGL)
	a = QApplication(sys.argv)
	w = OpenGLWindow()
	w.show()
	sys.exit(a.exec_())

if __name__ == "__main__":
	main()

