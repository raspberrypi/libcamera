#!/usr/bin/python3

import sys
import numpy as np
from OpenGL import GL as gl
from PyQt5.QtWidgets import QOpenGLWidget, QApplication
from PyQt5.QtGui import (QOpenGLBuffer, QOpenGLShaderProgram, QOpenGLShader)
from PyQt5.QtCore import Qt

class OpenGLWidget(QOpenGLWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Triangle, PyQt5, OpenGL ES 2.0")
        self.resize(400, 400)
    def initializeGL(self):
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
    w = OpenGLWidget()
    w.show()
    sys.exit(a.exec_())

if __name__ == "__main__":
    main()

