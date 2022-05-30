# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from helpers import mfb_to_rgb
from io import BytesIO
from PIL import Image
from PIL.ImageQt import ImageQt
from PyQt5 import QtCore, QtGui, QtWidgets
import libcamera as libcam
import libcamera.utils
import sys


def rgb_to_pix(rgb):
    img = Image.frombuffer('RGB', (rgb.shape[1], rgb.shape[0]), rgb)
    qim = ImageQt(img).copy()
    pix = QtGui.QPixmap.fromImage(qim)
    return pix


class QtRenderer:
    def __init__(self, state):
        self.state = state

        self.cm = state.cm
        self.contexts = state.contexts

    def setup(self):
        self.app = QtWidgets.QApplication([])

        windows = []

        for ctx in self.contexts:
            for stream in ctx.streams:
                window = MainWindow(ctx, stream)
                window.show()
                windows.append(window)

        self.windows = windows

    def run(self):
        camnotif = QtCore.QSocketNotifier(self.cm.event_fd, QtCore.QSocketNotifier.Read)
        camnotif.activated.connect(lambda _: self.readcam())

        keynotif = QtCore.QSocketNotifier(sys.stdin.fileno(), QtCore.QSocketNotifier.Read)
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
        buffers = req.buffers

        for stream, fb in buffers.items():
            wnd = next(wnd for wnd in self.windows if wnd.stream == stream)

            wnd.handle_request(stream, fb)

        self.state.request_processed(ctx, req)

    def cleanup(self):
        for w in self.windows:
            w.close()


class MainWindow(QtWidgets.QWidget):
    def __init__(self, ctx, stream):
        super().__init__()

        self.ctx = ctx
        self.stream = stream

        self.label = QtWidgets.QLabel()

        windowLayout = QtWidgets.QHBoxLayout()
        self.setLayout(windowLayout)

        windowLayout.addWidget(self.label)

        controlsLayout = QtWidgets.QVBoxLayout()
        windowLayout.addLayout(controlsLayout)

        windowLayout.addStretch()

        group = QtWidgets.QGroupBox('Info')
        groupLayout = QtWidgets.QVBoxLayout()
        group.setLayout(groupLayout)
        controlsLayout.addWidget(group)

        lab = QtWidgets.QLabel(ctx.id)
        groupLayout.addWidget(lab)

        self.frameLabel = QtWidgets.QLabel()
        groupLayout.addWidget(self.frameLabel)

        group = QtWidgets.QGroupBox('Properties')
        groupLayout = QtWidgets.QVBoxLayout()
        group.setLayout(groupLayout)
        controlsLayout.addWidget(group)

        camera = ctx.camera

        for cid, cv in camera.properties.items():
            lab = QtWidgets.QLabel()
            lab.setText('{} = {}'.format(cid, cv))
            groupLayout.addWidget(lab)

        group = QtWidgets.QGroupBox('Controls')
        groupLayout = QtWidgets.QVBoxLayout()
        group.setLayout(groupLayout)
        controlsLayout.addWidget(group)

        for cid, cinfo in camera.controls.items():
            lab = QtWidgets.QLabel()
            lab.setText('{} = {}/{}/{}'
                        .format(cid, cinfo.min, cinfo.max, cinfo.default))
            groupLayout.addWidget(lab)

        controlsLayout.addStretch()

    def buf_to_qpixmap(self, stream, fb):
        with libcamera.utils.MappedFrameBuffer(fb) as mfb:
            cfg = stream.configuration

            if cfg.pixel_format == libcam.formats.MJPEG:
                img = Image.open(BytesIO(mfb.planes[0]))
                qim = ImageQt(img).copy()
                pix = QtGui.QPixmap.fromImage(qim)
            else:
                rgb = mfb_to_rgb(mfb, cfg)
                if rgb is None:
                    raise Exception('Format not supported: ' + cfg.pixel_format)

                pix = rgb_to_pix(rgb)

        return pix

    def handle_request(self, stream, fb):
        ctx = self.ctx

        pix = self.buf_to_qpixmap(stream, fb)
        self.label.setPixmap(pix)

        self.frameLabel.setText('Queued: {}\nDone: {}\nFps: {:.2f}'
                                .format(ctx.reqs_queued, ctx.reqs_completed, ctx.fps))
