#!/usr/bin/python3 -i

from simplecamera import SimpleCameraManager, SimpleCamera
from PyQt5 import QtCore, QtGui, QtWidgets
import pycamera as pycam
import argparse
from io import BytesIO
from PIL import Image
from PIL.ImageQt import ImageQt
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("-c", "--cameras", type=str, default=None)
args = parser.parse_args()

format_map = {
	"YUYV": QtGui.QImage.Format.Format_RGB16,
	"MJPEG": QtGui.QImage.Format.Format_RGB888,
	"BGR888": QtGui.QImage.Format.Format_RGB888,
}


class MainWindow(QtWidgets.QWidget):
	requestDone = QtCore.pyqtSignal(pycam.Stream, pycam.FrameBuffer)

	def __init__(self, camera):
		super().__init__()

		# Use signal to handle request, so that the execution is transferred to the main thread
		self.requestDone.connect(self.handle_request)
		camera.callback = lambda stream, fb: self.requestDone.emit(stream, fb)

		camera.xxx_config()

		self.camera = camera

		self.label = QtWidgets.QLabel()

		windowLayout = QtWidgets.QHBoxLayout()
		self.setLayout(windowLayout)

		windowLayout.addWidget(self.label)

		controlsLayout = QtWidgets.QVBoxLayout()
		windowLayout.addLayout(controlsLayout)

		windowLayout.addStretch()

		group = QtWidgets.QGroupBox("Info")
		groupLayout = QtWidgets.QVBoxLayout()
		group.setLayout(groupLayout)
		controlsLayout.addWidget(group)

		lab = QtWidgets.QLabel(camera.id)
		groupLayout.addWidget(lab)

		self.frameLabel = QtWidgets.QLabel()
		groupLayout.addWidget(self.frameLabel)


		group = QtWidgets.QGroupBox("Properties")
		groupLayout = QtWidgets.QVBoxLayout()
		group.setLayout(groupLayout)
		controlsLayout.addWidget(group)

		for k, v in camera.properties.items():
			lab = QtWidgets.QLabel()
			lab.setText(k + " = " + str(v))
			groupLayout.addWidget(lab)

		group = QtWidgets.QGroupBox("Controls")
		groupLayout = QtWidgets.QVBoxLayout()
		group.setLayout(groupLayout)
		controlsLayout.addWidget(group)

		for k, (min, max, default) in camera.controls.items():
			lab = QtWidgets.QLabel()
			lab.setText("{} = {}/{}/{}".format(k, min, max, default))
			groupLayout.addWidget(lab)

		controlsLayout.addStretch()

		self.camera.start()

	def closeEvent(self, event):
		self.camera.stop()
		super().closeEvent(event)

	def buf_to_qpixmap(self, stream, fb):
		with fb.mmap(0) as b:
			cfg = stream.configuration
			qfmt = format_map[cfg.fmt]
			w, h = cfg.size
			pitch = cfg.stride

			if cfg.fmt == "MJPEG":
				img = Image.open(BytesIO(b))
				qim = ImageQt(img)
				pix = QtGui.QPixmap.fromImage(qim)
			elif cfg.fmt == "YUYV":
				arr = np.array(b)
				y = arr[0::2]
				u = arr[1::4]
				v = arr[3::4]

				yuv = np.ones((len(y)) * 3, dtype=np.uint8)
				yuv[::3] = y
				yuv[1::6] = u
				yuv[2::6] = v
				yuv[4::6] = u
				yuv[5::6] = v

				# XXX YCbCr doesn't work?
				#img = Image.frombytes("YCbCr", (w, h), yuv.tobytes())
				img = Image.frombuffer("RGB", (w, h), yuv)

				qim = ImageQt(img)
				pix = QtGui.QPixmap.fromImage(qim)
			else:
				img = QtGui.QImage(b, w, h, pitch, qfmt)
				pix = QtGui.QPixmap.fromImage(img)

		return pix

	def handle_request(self, stream, fb):
		global format_map

		#meta = fb.metadata
		#print("Buf seq {}, bytes {}".format(meta.sequence, meta.bytesused))

		pix = self.buf_to_qpixmap(stream, fb)
		self.label.setPixmap(pix)

		self.frameLabel.setText("Queued: {}\nDone: {}".format(camera.reqs_queued, camera.reqs_completed))

		self.camera.queue_fb(fb)


app = QtWidgets.QApplication([])
cm = SimpleCameraManager()

notif = QtCore.QSocketNotifier(cm.cm.efd, QtCore.QSocketNotifier.Read)
notif.activated.connect(lambda x: cm.read_events())

if not args.cameras:
	cameras = cm.cameras
else:
	cameras = []
	for name in args.cameras.split(","):
		c = cm.find(name)
		if not c:
			print("Camera not found: ", name)
			exit(-1)
		cameras.append(c)

windows = []

i = 0
for camera in cameras:
	globals()["cam" + str(i)] = camera
	i += 1

	camera.open()

	fmts = camera.formats

	for f in format_map:
		if f in fmts:
			camera.format = f
			break

	if not camera.format in format_map:
		raise Exception("Unsupported pixel format")

	camera.resolution = (640, 480)

	print("Capturing {}x{}-{}".format(camera.resolution[0], camera.resolution[1], camera.format))

	window = MainWindow(camera)
	window.setAttribute(QtCore.Qt.WA_ShowWithoutActivating)
	window.show()
	windows.append(window)

def cleanup():
	for w in windows:
		w.close()

	for camera in cameras:
		camera.close()
	print("Done")

import atexit
atexit.register(cleanup)
