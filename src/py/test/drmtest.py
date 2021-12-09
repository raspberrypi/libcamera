#!/usr/bin/python3

from simplecamera import SimpleCameraManager, SimpleCamera
import pykms
import pycamera as pycam
import time
import argparse
import selectors
import sys
import threading

class ScreenHandler:
	def __init__(self):
		card = pykms.Card()

		res = pykms.ResourceManager(card)
		conn = res.reserve_connector()
		crtc = res.reserve_crtc(conn)
		plane = res.reserve_generic_plane(crtc)
		mode = conn.get_default_mode()
		modeb = mode.to_blob(card)

		req = pykms.AtomicReq(card)
		req.add_connector(conn, crtc)
		req.add_crtc(crtc, modeb)
		req.commit_sync(allow_modeset = True)

		self.card = card
		self.crtc = crtc
		self.plane = plane
		self.bufqueue = []
		self.current = None
		self.next = None

	def close(self):
		req = pykms.AtomicReq(self.card)
		req.add_plane(self.plane, None, None, dst=(0, 0, 0, 0))
		req.commit()

		self.card = None

	def handle_page_flip(self, frame, time):
		old = self.current
		self.current = self.next

		if len(self.bufqueue) > 0:
			self.next = self.bufqueue.pop(0)
		else:
			self.next = None

		if self.next:
			req = pykms.AtomicReq(self.card)
			req.add_plane(self.plane, fb, self.crtc, dst=(0, 0, fb.width, fb.height))
			req.commit()

		return old

	def queue(self, fb):
		if not self.next:
			self.next = fb

			req = pykms.AtomicReq(self.card)
			req.add_plane(self.plane, fb, self.crtc, dst=(0, 0, fb.width, fb.height))
			req.commit()
		else:
			self.bufqueue.append(fb)



screen = None
cm = None
cam = None
cam_2_drm_map = {}
drm_2_cam_map = {}

def handle_camera_frame(camera, stream, fb):
	screen.queue(cam_2_drm_map[fb])

def setup():
	global cm
	global cam
	global cam_2_drm_map
	global drm_2_cam_map

	cm = SimpleCameraManager()
	cam = cm.find("vimc")
	cam.open()

	cam.format = "BGR888"
	cam.resolution = (800, 600)

	cam.callback = lambda stream, fb, camera=cam: handle_camera_frame(camera, stream, fb)

	cam.xxx_config()

	drmbuffers = []
	stream_cfg = cam.stream_config

	for fb in cam.buffers:
		w, h = stream_cfg.size
		stride = stream_cfg.stride
		fd = fb.fd(0)
		drmfb = pykms.DmabufFramebuffer(screen.card, w, h, pykms.PixelFormat.RGB888,
										[fd], [stride], [0])
		drmbuffers.append(drmfb)

		cam_2_drm_map[fb] = drmfb
		drm_2_cam_map[drmfb] = fb

def readdrm(fileobj, mask):
	for ev in screen.card.read_events():
		if ev.type == pykms.DrmEventType.FLIP_COMPLETE:
			old = screen.handle_page_flip(ev.seq, ev.time)

			if old:
				fb = drm_2_cam_map[old]
				cam.queue_fb(fb)

def readcam(fileobj, mask):
	cm.read_events()

running = True

def readkey(fileobj, mask):
	global running
	sys.stdin.readline()
	running = False

def thread_run():
	global running
	global screen

	screen = ScreenHandler()

	setup();

	sel = selectors.DefaultSelector()
	sel.register(screen.card.fd, selectors.EVENT_READ, readdrm)
	#sel.register(sys.stdin, selectors.EVENT_READ, readkey)
	sel.register(cm.cm.efd, selectors.EVENT_READ, readcam)

	cam.start()

	while running:
		events = sel.select()
		for key, mask in events:
			callback = key.data
			callback(key.fileobj, mask)

	cam.stop()

	screen.close()
	screen = None

preview_thread = None

def start_preview():
	global preview_thread
	global running

	running = True

	preview_thread = threading.Thread(target=thread_run, args=[])
	preview_thread.start()

def stop_preview():
	global preview_thread
	global running

	running = False

	preview_thread.join()

start_preview()

#print("Press enter to exit")
#
#while running:
#	events = sel.select()
#	for key, mask in events:
#		callback = key.data
#		callback(key.fileobj, mask)

#cam.stop()

#print("Done")
