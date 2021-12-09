#!/usr/bin/python3

from simplecamera import SimpleCameraManager, SimpleCamera
import selectors
import sys

def handle_camera_frame(camera, stream, fb):
	print("buf", fb)
	cam.queue_fb(fb)

cm = SimpleCameraManager()
cam = cm.find("imx219")
cam.open()

cam.format = "XRGB8888"
cam.resolution = (1920, 1080)

cam.callback = lambda stream, fb, camera=cam: handle_camera_frame(camera, stream, fb)

cam.xxx_config()

cam.start()

def readcam(fileobj, mask):
	cm.read_events()

running = True

def readkey(fileobj, mask):
	global running
	sys.stdin.readline()
	running = False

sel = selectors.DefaultSelector()
sel.register(sys.stdin, selectors.EVENT_READ, readkey)
sel.register(cm.cm.efd, selectors.EVENT_READ, readcam)

print("Press enter to exit")

while running:
	events = sel.select()
	for key, mask in events:
		callback = key.data
		callback(key.fileobj, mask)

cam.stop()

print("Done")
