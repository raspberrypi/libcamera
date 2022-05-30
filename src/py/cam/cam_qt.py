# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from helpers import mfb_to_rgb
from PyQt5 import QtCore, QtGui, QtWidgets
import libcamera as libcam
import libcamera.utils
import sys


# Loading MJPEG to a QPixmap produces corrupt JPEG data warnings. Ignore these.
def qt_message_handler(msg_type, msg_log_context, msg_string):
    if msg_string.startswith("Corrupt JPEG data"):
        return

    # For some reason qInstallMessageHandler returns None, so we won't
    # call the old handler
    if old_msg_handler is not None:
        old_msg_handler(msg_type, msg_log_context, msg_string)
    else:
        print(msg_string)


old_msg_handler = QtCore.qInstallMessageHandler(qt_message_handler)


def rgb_to_pix(rgb):
    w = rgb.shape[1]
    h = rgb.shape[0]
    qim = QtGui.QImage(rgb, w, h, QtGui.QImage.Format.Format_RGB888)
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

        buf_mmap_map = {}

        for ctx in self.contexts:
            for stream in ctx.streams:
                for buf in ctx.allocator.buffers(stream):
                    mfb = libcamera.utils.MappedFrameBuffer(buf).mmap()
                    buf_mmap_map[buf] = mfb

        self.buf_mmap_map = buf_mmap_map

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

            mfb = self.buf_mmap_map[fb]

            wnd.handle_request(stream, mfb)

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

    def buf_to_qpixmap(self, stream, mfb):
        cfg = stream.configuration

        if cfg.pixel_format == libcam.formats.MJPEG:
            pix = QtGui.QPixmap(cfg.size.width, cfg.size.height)
            pix.loadFromData(mfb.planes[0])
        else:
            rgb = mfb_to_rgb(mfb, cfg)
            if rgb is None:
                raise Exception('Format not supported: ' + cfg.pixel_format)

            pix = rgb_to_pix(rgb)

        return pix

    def handle_request(self, stream, mfb):
        ctx = self.ctx

        pix = self.buf_to_qpixmap(stream, mfb)
        self.label.setPixmap(pix)

        self.frameLabel.setText('Queued: {}\nDone: {}\nFps: {:.2f}'
                                .format(ctx.reqs_queued, ctx.reqs_completed, ctx.fps))
