#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

# A simple capture example extending the simple-capture.py example:
# - Capture frames using events from multiple cameras
# - Listening events from stdin to exit the application
# - Memory mapping the frames and calculating CRC

import binascii
import libcamera as libcam
import libcamera.utils
import selectors
import sys


# A container class for our state per camera
class CameraCaptureContext:
    idx: int
    cam: libcam.Camera
    reqs: list[libcam.Request]
    mfbs: dict[libcam.FrameBuffer, libcamera.utils.MappedFrameBuffer]

    def __init__(self, cam, idx):
        self.idx = idx
        self.cam = cam

        # Acquire the camera for our use

        ret = cam.acquire()
        assert ret == 0

        # Configure the camera

        cam_config = cam.generate_configuration([libcam.StreamRole.Viewfinder])

        stream_config = cam_config.at(0)

        ret = cam.configure(cam_config)
        assert ret == 0

        stream = stream_config.stream

        # Allocate the buffers for capture

        allocator = libcam.FrameBufferAllocator(cam)
        ret = allocator.allocate(stream)
        assert ret > 0

        num_bufs = len(allocator.buffers(stream))

        print(f'cam{idx} ({cam.id}): capturing {num_bufs} buffers with {stream_config}')

        # Create the requests and assign a buffer for each request

        self.reqs = []
        self.mfbs = {}

        for i in range(num_bufs):
            # Use the buffer index as the "cookie"
            req = cam.create_request(idx)

            buffer = allocator.buffers(stream)[i]
            ret = req.add_buffer(stream, buffer)
            assert ret == 0

            self.reqs.append(req)

            # Save a mmapped buffer so we can calculate the CRC later
            self.mfbs[buffer] = libcamera.utils.MappedFrameBuffer(buffer).mmap()

    def uninit_camera(self):
        # Stop the camera

        ret = self.cam.stop()
        assert ret == 0

        # Release the camera

        ret = self.cam.release()
        assert ret == 0


# A container class for our state
class CaptureContext:
    cm: libcam.CameraManager
    camera_contexts: list[CameraCaptureContext] = []

    def handle_camera_event(self):
        # cm.get_ready_requests() returns the ready requests, which in our case
        # should almost always return a single Request, but in some cases there
        # could be multiple or none.

        reqs = self.cm.get_ready_requests()

        # Process the captured frames

        for req in reqs:
            self.handle_request(req)

        return True

    def handle_request(self, req: libcam.Request):
        cam_ctx = self.camera_contexts[req.cookie]

        buffers = req.buffers

        assert len(buffers) == 1

        # A ready Request could contain multiple buffers if multiple streams
        # were being used. Here we know we only have a single stream,
        # and we use next(iter()) to get the first and only buffer.

        stream, fb = next(iter(buffers.items()))

        # Use the MappedFrameBuffer to access the pixel data with CPU. We calculate
        # the crc for each plane.

        mfb = cam_ctx.mfbs[fb]
        crcs = [binascii.crc32(p) for p in mfb.planes]

        meta = fb.metadata

        print('cam{:<6} seq {:<6} bytes {:10} CRCs {}'
              .format(cam_ctx.idx,
                      meta.sequence,
                      '/'.join([str(p.bytes_used) for p in meta.planes]),
                      crcs))

        # We want to re-queue the buffer we just handled. Instead of creating
        # a new Request, we re-use the old one. We need to call req.reuse()
        # to re-initialize the Request before queuing.

        req.reuse()
        cam_ctx.cam.queue_request(req)

    def handle_key_event(self):
        sys.stdin.readline()
        print('Exiting...')
        return False

    def capture(self):
        # Queue the requests to the camera

        for cam_ctx in self.camera_contexts:
            for req in cam_ctx.reqs:
                ret = cam_ctx.cam.queue_request(req)
                assert ret == 0

        # Use Selector to wait for events from the camera and from the keyboard

        sel = selectors.DefaultSelector()
        sel.register(sys.stdin, selectors.EVENT_READ, self.handle_key_event)
        sel.register(self.cm.event_fd, selectors.EVENT_READ, lambda: self.handle_camera_event())

        running = True

        while running:
            events = sel.select()
            for key, mask in events:
                # If the handler return False, we should exit
                if not key.data():
                    running = False


def main():
    cm = libcam.CameraManager.singleton()

    ctx = CaptureContext()
    ctx.cm = cm

    for idx, cam in enumerate(cm.cameras):
        cam_ctx = CameraCaptureContext(cam, idx)
        ctx.camera_contexts.append(cam_ctx)

    # Start the cameras

    for cam_ctx in ctx.camera_contexts:
        ret = cam_ctx.cam.start()
        assert ret == 0

    ctx.capture()

    for cam_ctx in ctx.camera_contexts:
        cam_ctx.uninit_camera()

    return 0


if __name__ == '__main__':
    sys.exit(main())
