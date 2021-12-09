import pycamera as pycam
import os

class SimpleCameraManager:
	def __init__(self):
		self.cm = pycam.CameraManager.singleton()

		self.cameras = []
		for c in self.cm.cameras:
			self.cameras.append(SimpleCamera(c))

	def find(self, name):
		for c in self.cameras:
			if name.lower() in c.id.lower():
				return c

		return None

	def read_events(self):
		data = os.read(self.cm.efd, 8)

		reqs = self.cm.getReadyRequests()

		for req in reqs:
			for c in self.cameras:
				if c.pycam == req.camera:
					c.req_complete_cb(req)

class SimpleCamera:
	def __init__(self, camera):
		self.pycam = camera

		self.callback = None

		self.control_values = {}
		#for k, (min, max, default) in self.pycam.controls.items():
		#	self.control_values[k] = default

		self.running = False

	def __repr__(self):
		return "<SimpleCamera '" + self.pycam.id + "'>"

	@property
	def id(self):
		return self.pycam.id

	@property
	def formats(self):
		return self.stream_config.formats.pixelFormats

	def open(self):
		self.pycam.acquire()

		self.camera_config = self.pycam.generateConfiguration([pycam.StreamRole.Viewfinder])
		self.stream_config = self.camera_config.at(0)

	def close(self):
		self.pycam.release()

	@property
	def properties(self):
		return self.pycam.properties

	@property
	def controls(self):
		return self.pycam.controls

	def xxx_config(self):
		self.configure_camera()
		self.alloc_buffers()

	def start(self):
		self.reqs_queued = 0
		self.reqs_completed = 0

		self.running = True
		self.pycam.start()

		self.queue_initial_fbs()

	def stop(self):
		self.running = False

		self.pycam.stop()

		self.buffers = None

	@property
	def resolution(self):
		return self.stream_config.size

	@resolution.setter
	def resolution(self, val):
		running = self.running
		if running:
			self.stop()

		self.stream_config.size = val
		self.camera_config.validate()

		if running:
			self.start()

	@property
	def format(self):
		return self.stream_config.fmt

	@format.setter
	def format(self, val):
		running = self.running
		if running:
			self.stop()

		self.stream_config.fmt = val
		self.camera_config.validate()

		if running:
			self.start()

	@property
	def bufferCount(self):
		return self.stream_config.bufferCount

	@bufferCount.setter
	def bufferCount(self, val):
		running = self.running
		if running:
			self.stop()

		self.stream_config.bufferCount = val
		self.camera_config.validate()

		if running:
			self.start()

	def configure_camera(self):
		camera = self.pycam

		status = self.camera_config.validate()

		if status == pycam.ConfigurationStatus.Invalid:
			raise Exception("Invalid configuration")

		print("Cam: config {}".format(self.stream_config.toString()))

		camera.configure(self.camera_config);

	def alloc_buffers(self):
		camera = self.pycam
		stream = self.stream_config.stream

		allocator = pycam.FrameBufferAllocator(camera);
		ret = allocator.allocate(stream)
		if ret < 0:
			raise Exception("Can't allocate buffers")

		self.buffers = allocator.buffers(stream)

		print("Cam: Allocated {} buffers for stream".format(len(self.buffers)))

	def queue_initial_fbs(self):
		buffers = self.buffers

		for fb in buffers:
			self.queue_fb(fb)

	def queue_fb(self, fb):
		camera = self.pycam
		stream = self.stream_config.stream

		request = camera.createRequest()

		if request == None:
			raise Exception("Can't create request")

		ret = request.addBuffer(stream, fb)
		if ret < 0:
			raise Exception("Can't set buffer for request")

		# XXX: ExposureTime cannot be set if AeEnable == True
		skip_exp_time = "AeEnable" in self.control_values and self.control_values["AeEnable"] == True

		for k, v in self.control_values.items():
			if k == "ExposureTime" and skip_exp_time:
				continue
			request.set_control(k, v)

		control_values = {}

		camera.queueRequest(request)

		self.reqs_queued += 1


	def req_complete_cb(self, req):
		camera = self.pycam

		assert(len(req.buffers) == 1)

		stream, fb = next(iter(req.buffers.items()))

		self.reqs_completed += 1

		if self.running and self.callback:
			self.callback(stream, fb)

	def set_control(self, control, value):
		if not control in self.pycam.controls:
			for k in self.pycam.controls:
				if control.lower() == k.lower():
					control = k

		self.control_values[control] = value
