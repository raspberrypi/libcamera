#!/usr/bin/python3

import pycamera as pycam
import time
import binascii
import argparse
import selectors
import os

parser = argparse.ArgumentParser()
parser.add_argument("-n", "--num-frames", type=int, default=10)
parser.add_argument("-c", "--print-crc", action="store_true")
parser.add_argument("-s", "--save-frames", action="store_true")
args = parser.parse_args()

CONFIG = "vivid"

CONFIGS = {}

CONFIGS["rpi"] = [ {
	"device": "/base/soc/i2c0mux/i2c@1/imx219@10",
	"streams": [
		{
			"role": pycam.StreamRole.Viewfinder,
			"size": (800, 600),
			"fmt": "RGB888",
		}, {
			"role": pycam.StreamRole.VideoRecording,
			"size": (800, 600),
			"fmt": "NV12",
		},
	],
} ]

CONFIGS["uvc"] = [ {
	"device": "\\_SB_.PCI0.EHC2.",
	"streams": [
		{
			"role": pycam.StreamRole.Viewfinder,
			"size": (800, 600),
			"fmt": "YUYV",
		},
	],
} ]

CONFIGS["laptop"] = [ {
	"device": "\\_SB_.PCI0.EHC1.",
	"streams": [
		{
			"role": pycam.StreamRole.Viewfinder,
			"size": (640, 480),
			"fmt": "YUYV",
		},
	],
} ]

CONFIGS["vivid"] = [ {
	"device": "vivid",
	"streams": [
		{
			"role": pycam.StreamRole.Viewfinder,
			"size": (640, 480),
			"fmt": "XRGB8888",
		},
	],
} ]

config = CONFIGS[CONFIG]

cm = pycam.CameraManager.singleton()

cameras = []

for conf in config:
	cam = cm.find(conf["device"])
	assert(cam != None)
	cameras.append(cam)

if len(cameras) == 0:
	print("No cameras")
	exit(0)

print("Cameras:")
for c in cameras:
	print("    {}".format(c.id))
	print("        Properties:", c.properties)
	print("        Controls:", c.controls)

contexts = []

for i in range(len(cameras)):
	contexts.append({ "id": i, "camera": cameras[i], "config": config[i] })

for ctx in contexts:
	ctx["camera"].acquire()

# xxx move to c++ side?
def do_config(camera, stream_configs):
	roles = [ s["role"] for s in stream_configs ]

	camconfig = camera.generateConfiguration(roles)
	if camconfig == None:
		raise Exception("Generating config failed")

	for i, stream_config in enumerate(camconfig):
		cfg = stream_configs[i]

		stream_config.size = cfg["size"]
		stream_config.fmt = cfg["fmt"]

	stat = camconfig.validate()
	if stat != pycam.ConfigurationStatus.Valid:
		raise Exception("Invalid config")

	r = camera.configure(camconfig);
	if r != 0:
		raise Exception("Configure failed")

	streams = []

	for stream_config in camconfig:
		stream = stream_config.stream
		streams.append(stream)

	return streams


def configure_camera(ctx):
	camera = ctx["camera"]

	streams = do_config(camera, ctx["config"]["streams"])

	for stream in streams:
		print("Cam {}: stream config {}".format(ctx["id"], stream.configuration.toString()))

	ctx["streams"] = streams

def alloc_buffers(ctx):
	camera = ctx["camera"]

	allocator = pycam.FrameBufferAllocator(camera);

	for stream in ctx["streams"]:
		ret = allocator.allocate(stream)
		if ret < 0:
			print("Can't allocate buffers")
			exit(-1)

		allocated = len(allocator.buffers(stream))
		print("Cam {}: Allocated {} buffers for stream {}".format(ctx["id"], allocated, i))

	ctx["allocator"] = allocator

def create_requests(ctx):
	camera = ctx["camera"]

	ctx["requests"] = []

	# Identify the stream with the least number of buffers
	num_bufs = min([len(ctx["allocator"].buffers(stream)) for stream in ctx["streams"]])

	requests = []

	for buf_num in range(num_bufs):
		request = camera.createRequest()

		if request == None:
			print("Can't create request")
			exit(-1)

		for stream in ctx["streams"]:
			buffers = ctx["allocator"].buffers(stream)
			buffer = buffers[buf_num]

			ret = request.addBuffer(stream, buffer)
			if ret < 0:
				print("Can't set buffer for request")
				exit(-1)

			requests.append(request)

	ctx["requests"] = requests


def req_complete_cb(ctx, req):
	camera = ctx["camera"]

	bufs = req.buffers

	print("Cam {}: Req {} Complete: {} numbufs {}".format(ctx["id"], ctx["reqs_completed"], req.status, len(bufs)))

	for stream, fb in bufs.items():
		meta = fb.metadata
		print("Cam {}: Buf seq {}, bytes {}".format(ctx["id"], meta.sequence, meta.bytesused))

		with fb.mmap(0) as b:
			if args.print_crc:
				crc = binascii.crc32(b)
				print("Cam {}:    CRC {:#x}".format(ctx["id"], crc))

			if args.save_frames:
				id = ctx["id"]
				num = ctx["reqs_completed"]
				filename = "frame-{}-{}.data".format(id, num)
				with open(filename, "wb") as f:
					f.write(b)
				print("Cam {}:    Saved {}".format(ctx["id"], filename))

	ctx["reqs_completed"] += 1

	if ctx["reqs_queued"] < args.num_frames:
		req.reuse()
		camera.queueRequest(req)
		ctx["reqs_queued"] += 1


def setup_callbacks(ctx):
	camera = ctx["camera"]

	ctx["reqs_queued"] = 0
	ctx["reqs_completed"] = 0

def queue_requests(ctx):
	camera = ctx["camera"]

	camera.start()

	for request in ctx["requests"]:
		camera.queueRequest(request)
		ctx["reqs_queued"] += 1

	del ctx["requests"]

for ctx in contexts:
	configure_camera(ctx)
	alloc_buffers(ctx)
	create_requests(ctx)
	setup_callbacks(ctx)

for ctx in contexts:
	queue_requests(ctx)


print("Processing...")

running = True

def readcam(fileobj, mask):
	global running
	data = os.read(fileobj, 8)

	reqs = cm.getReadyRequests()

	ctx = contexts[0]
	for req in reqs:
		ctx = next(ctx for ctx in contexts if ctx["camera"] == req.camera)
		req_complete_cb(ctx, req)

	running =  any(ctx["reqs_completed"] < args.num_frames for ctx in contexts)


sel = selectors.DefaultSelector()
sel.register(cm.efd, selectors.EVENT_READ, readcam)

print("Press enter to exit")

while running:
	events = sel.select()
	for key, mask in events:
		callback = key.data
		callback(key.fileobj, mask)

print("Exiting...")

for ctx in contexts:
	camera = ctx["camera"]
	camera.stop()
	camera.release()

print("Done")
