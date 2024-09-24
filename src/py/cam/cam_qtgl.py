# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from PyQt6 import QtCore, QtWidgets
from PyQt6.QtCore import Qt

import math
import os
import sys

os.environ['PYOPENGL_PLATFORM'] = 'egl'

from OpenGL.EGL.EXT.image_dma_buf_import import *
from OpenGL.EGL.KHR.image import *
from OpenGL.EGL.VERSION.EGL_1_0 import *
from OpenGL.EGL.VERSION.EGL_1_2 import *
from OpenGL.EGL.VERSION.EGL_1_3 import *

from OpenGL.GLES2.OES.EGL_image import *
from OpenGL.GLES2.OES.EGL_image_external import *
from OpenGL.GLES2.VERSION.GLES2_2_0 import *
from OpenGL.GLES3.VERSION.GLES3_3_0 import *

from OpenGL.GL import shaders

from gl_helpers import *


class EglState:
    def __init__(self):
        self.create_display()
        self.choose_config()
        self.create_context()
        self.check_extensions()

    def create_display(self):
        xdpy = getEGLNativeDisplay()
        dpy = eglGetDisplay(xdpy)
        self.display = dpy

    def choose_config(self):
        dpy = self.display

        major, minor = EGLint(), EGLint()

        b = eglInitialize(dpy, major, minor)
        assert(b)

        print('EGL {} {}'.format(
              eglQueryString(dpy, EGL_VENDOR).decode(),
              eglQueryString(dpy, EGL_VERSION).decode()))

        check_egl_extensions(dpy, ['EGL_EXT_image_dma_buf_import'])

        b = eglBindAPI(EGL_OPENGL_ES_API)
        assert(b)

        def print_config(dpy, cfg):

            def getconf(a):
                value = ctypes.c_long()
                eglGetConfigAttrib(dpy, cfg, a, value)
                return value.value

            print('EGL Config {}: color buf {}/{}/{}/{} = {}, depth {}, stencil {}, native visualid {}, native visualtype {}'.format(
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
            eglGetConfigs(dpy, None, 0, num_configs)
            print('{} configs'.format(num_configs.value))

            configs = (EGLConfig * num_configs.value)()
            eglGetConfigs(dpy, configs, num_configs.value, num_configs)
            for config_id in configs:
                print_config(dpy, config_id)

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
        b = eglChooseConfig(dpy, config_attribs, configs, 1, n)
        assert(b and n.value == 1)
        config = configs[0]

        print('Chosen Config:')
        print_config(dpy, config)

        self.config = config

    def create_context(self):
        dpy = self.display

        context_attribs = [
            EGL_CONTEXT_CLIENT_VERSION, 2,
            EGL_NONE,
        ]

        context = eglCreateContext(dpy, self.config, EGL_NO_CONTEXT, context_attribs)
        assert(context)

        b = eglMakeCurrent(dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, context)
        assert(b)

        self.context = context

    def check_extensions(self):
        check_gl_extensions(['GL_OES_EGL_image'])

        assert(eglCreateImageKHR)
        assert(eglDestroyImageKHR)
        assert(glEGLImageTargetTexture2DOES)


class QtRenderer:
    def __init__(self, state):
        self.state = state

    def setup(self):
        self.app = QtWidgets.QApplication([])

        window = MainWindow(self.state)
        window.show()

        self.window = window

    def run(self):
        camnotif = QtCore.QSocketNotifier(self.state.cm.event_fd, QtCore.QSocketNotifier.Type.Read)
        camnotif.activated.connect(lambda _: self.readcam())

        keynotif = QtCore.QSocketNotifier(sys.stdin.fileno(), QtCore.QSocketNotifier.Type.Read)
        keynotif.activated.connect(lambda _: self.readkey())

        print('Capturing...')

        self.app.exec()

        print('Exiting...')

    def readcam(self):
        running = self.state.event_handler()

        if not running:
            self.app.quit()

    def readkey(self):
        sys.stdin.readline()
        self.app.quit()

    def request_handler(self, ctx, req):
        self.window.handle_request(ctx, req)

    def cleanup(self):
        self.window.close()


class MainWindow(QtWidgets.QWidget):
    def __init__(self, state):
        super().__init__()

        self.setAttribute(Qt.WidgetAttribute.WA_PaintOnScreen)
        self.setAttribute(Qt.WidgetAttribute.WA_NativeWindow)

        self.state = state

        self.textures = {}
        self.reqqueue = {}
        self.current = {}

        for ctx in self.state.contexts:

            self.reqqueue[ctx.idx] = []
            self.current[ctx.idx] = []

            for stream in ctx.streams:
                self.textures[stream] = None

        num_tiles = len(self.textures)
        self.num_columns = math.ceil(math.sqrt(num_tiles))
        self.num_rows = math.ceil(num_tiles / self.num_columns)

        self.egl = EglState()

        self.surface = None

    def paintEngine(self):
        return None

    def create_surface(self):
        native_surface = c_void_p(self.winId().__int__())
        surface = eglCreateWindowSurface(self.egl.display, self.egl.config,
                                         native_surface, None)

        b = eglMakeCurrent(self.egl.display, self.surface, self.surface, self.egl.context)
        assert(b)

        self.surface = surface

    def init_gl(self):
        self.create_surface()

        vertShaderSrc = '''
            attribute vec2 aPosition;
            varying vec2 texcoord;

            void main()
            {
                gl_Position = vec4(aPosition * 2.0 - 1.0, 0.0, 1.0);
                texcoord.x = aPosition.x;
                texcoord.y = 1.0 - aPosition.y;
            }
        '''
        fragShaderSrc = '''
            #extension GL_OES_EGL_image_external : enable
            precision mediump float;
            varying vec2 texcoord;
            uniform samplerExternalOES texture;

            void main()
            {
                gl_FragColor = texture2D(texture, texcoord);
            }
        '''

        program = shaders.compileProgram(
            shaders.compileShader(vertShaderSrc, GL_VERTEX_SHADER),
            shaders.compileShader(fragShaderSrc, GL_FRAGMENT_SHADER)
        )

        glUseProgram(program)

        glClearColor(0.5, 0.8, 0.7, 1.0)

        vertPositions = [
            0.0, 0.0,
            1.0, 0.0,
            1.0, 1.0,
            0.0, 1.0
        ]

        inputAttrib = glGetAttribLocation(program, 'aPosition')
        glVertexAttribPointer(inputAttrib, 2, GL_FLOAT, GL_FALSE, 0, vertPositions)
        glEnableVertexAttribArray(inputAttrib)

    def create_texture(self, stream, fb):
        cfg = stream.configuration
        fmt = cfg.pixel_format.fourcc
        w = cfg.size.width
        h = cfg.size.height

        attribs = [
            EGL_WIDTH, w,
            EGL_HEIGHT, h,
            EGL_LINUX_DRM_FOURCC_EXT, fmt,
            EGL_DMA_BUF_PLANE0_FD_EXT, fb.planes[0].fd,
            EGL_DMA_BUF_PLANE0_OFFSET_EXT, 0,
            EGL_DMA_BUF_PLANE0_PITCH_EXT, cfg.stride,
            EGL_NONE,
        ]

        image = eglCreateImageKHR(self.egl.display,
                                  EGL_NO_CONTEXT,
                                  EGL_LINUX_DMA_BUF_EXT,
                                  None,
                                  attribs)
        assert(image)

        textures = glGenTextures(1)
        glBindTexture(GL_TEXTURE_EXTERNAL_OES, textures)
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_EXTERNAL_OES, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glEGLImageTargetTexture2DOES(GL_TEXTURE_EXTERNAL_OES, image)

        return textures

    def resizeEvent(self, event):
        size = event.size()

        print('Resize', size)

        super().resizeEvent(event)

        if self.surface is None:
            return

        glViewport(0, 0, size.width() // 2, size.height())

    def paintEvent(self, event):
        if self.surface is None:
            self.init_gl()

        for ctx_idx, queue in self.reqqueue.items():
            if len(queue) == 0:
                continue

            ctx = next(ctx for ctx in self.state.contexts if ctx.idx == ctx_idx)

            if self.current[ctx_idx]:
                old = self.current[ctx_idx]
                self.current[ctx_idx] = None
                self.state.request_processed(ctx, old)

            next_req = queue.pop(0)
            self.current[ctx_idx] = next_req

            stream, fb = next(iter(next_req.buffers.items()))

            self.textures[stream] = self.create_texture(stream, fb)

        self.paint_gl()

    def paint_gl(self):
        b = eglMakeCurrent(self.egl.display, self.surface, self.surface, self.egl.context)
        assert(b)

        glClear(GL_COLOR_BUFFER_BIT)

        size = self.size()

        for idx, ctx in enumerate(self.state.contexts):
            for stream in ctx.streams:
                if self.textures[stream] is None:
                    continue

                w = size.width() // self.num_columns
                h = size.height() // self.num_rows

                x = idx % self.num_columns
                y = idx // self.num_columns

                x *= w
                y *= h

                glViewport(x, y, w, h)

                glBindTexture(GL_TEXTURE_EXTERNAL_OES, self.textures[stream])
                glDrawArrays(GL_TRIANGLE_FAN, 0, 4)

        b = eglSwapBuffers(self.egl.display, self.surface)
        assert(b)

    def handle_request(self, ctx, req):
        self.reqqueue[ctx.idx].append(req)
        self.update()
