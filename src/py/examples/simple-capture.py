#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

# A simple capture example showing:
# - How to setup the camera
# - Capture certain number of frames in a blocking manner
# - How to stop the camera
#
# This simple example is, in many ways, too simple. The purpose of the example
# is to introduce the concepts. A more realistic example is given in
# simple-continuous-capture.py.

import argparse
import libcamera as libcam
import selectors
import sys

# Number of frames to capture
TOTAL_FRAMES = 30


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--camera', type=str, default='1',
                        help='Camera index number (starting from 1) or part of the name')
    parser.add_argument('-f', '--format', type=str, help='Pixel format')
    parser.add_argument('-s', '--size', type=str, help='Size ("WxH")')
    args = parser.parse_args()

    cm = libcam.CameraManager.singleton()

    try:
        if args.camera.isnumeric():
            cam_idx = int(args.camera)
            cam = next((cam for i, cam in enumerate(cm.cameras) if i + 1 == cam_idx))
        else:
            cam = next((cam for cam in cm.cameras if args.camera in cam.id))
    except Exception:
        print(f'Failed to find camera "{args.camera}"')
        return -1

    # Acquire the camera for our use

    ret = cam.acquire()
    assert ret == 0

    # Configure the camera

    cam_config = cam.generate_configuration([libcam.StreamRole.Viewfinder])

    stream_config = cam_config.at(0)

    if args.format:
        fmt = libcam.PixelFormat(args.format)
        stream_config.pixel_format = fmt

    if args.size:
        w, h = [int(v) for v in args.size.split('x')]
        stream_config.size = libcam.Size(w, h)

    ret = cam.configure(cam_config)
    assert ret == 0

    print(f'Capturing {TOTAL_FRAMES} frames with {stream_config}')

    stream = stream_config.stream

    # Allocate the buffers for capture

    allocator = libcam.FrameBufferAllocator(cam)
    ret = allocator.allocate(stream)
    assert ret > 0

    num_bufs = len(allocator.buffers(stream))

    # Create the requests and assign a buffer for each request

    reqs = []
    for i in range(num_bufs):
        # Use the buffer index as the cookie
        req = cam.create_request(i)

        buffer = allocator.buffers(stream)[i]
        ret = req.add_buffer(stream, buffer)
        assert ret == 0

        reqs.append(req)

    # Start the camera

    ret = cam.start()
    assert ret == 0

    # frames_queued and frames_done track the number of frames queued and done

    frames_queued = 0
    frames_done = 0

    # Queue the requests to the camera

    for req in reqs:
        ret = cam.queue_request(req)
        assert ret == 0
        frames_queued += 1

    # The main loop. Wait for the queued Requests to complete, process them,
    # and re-queue them again.

    sel = selectors.DefaultSelector()
    sel.register(cm.event_fd, selectors.EVENT_READ)

    while frames_done < TOTAL_FRAMES:
        # cm.get_ready_requests() does not block, so we use a Selector to wait
        # for a camera event. Here we should almost always get a single
        # Request, but in some cases there could be multiple or none.

        events = sel.select()
        if not events:
            continue

        reqs = cm.get_ready_requests()

        for req in reqs:
            frames_done += 1

            buffers = req.buffers

            # A ready Request could contain multiple buffers if multiple streams
            # were being used. Here we know we only have a single stream,
            # and we use next(iter()) to get the first and only buffer.

            assert len(buffers) == 1

            stream, fb = next(iter(buffers.items()))

            # Here we could process the received buffer. In this example we only
            # print a few details below.

            meta = fb.metadata

            print("seq {:3}, bytes {}, frames queued/done {:3}/{:<3}"
                  .format(meta.sequence,
                          '/'.join([str(p.bytes_used) for p in meta.planes]),
                          frames_queued, frames_done))

            # If we want to capture more frames we need to queue more Requests.
            # We could create a totally new Request, but it is more efficient
            # to reuse the existing one that we just received.
            if frames_queued < TOTAL_FRAMES:
                req.reuse()
                cam.queue_request(req)
                frames_queued += 1

    # Stop the camera

    ret = cam.stop()
    assert ret == 0

    # Release the camera

    ret = cam.release()
    assert ret == 0

    return 0


if __name__ == '__main__':
    sys.exit(main())
