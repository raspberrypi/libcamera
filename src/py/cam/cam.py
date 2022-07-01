#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from typing import Any
import argparse
import binascii
import libcamera as libcam
import libcamera.utils
import sys
import traceback


class CameraContext:
    camera: libcam.Camera
    id: str
    idx: int

    opt_stream: str
    opt_strict_formats: bool
    opt_crc: bool
    opt_metadata: bool
    opt_save_frames: bool
    opt_capture: int

    stream_names: dict[libcam.Stream, str]
    streams: list[libcam.Stream]
    allocator: libcam.FrameBufferAllocator
    requests: list[libcam.Request]
    reqs_queued: int
    reqs_completed: int
    last: int = 0
    fps: float

    def __init__(self, camera, idx):
        self.camera = camera
        self.idx = idx
        self.id = 'cam' + str(idx)
        self.reqs_queued = 0
        self.reqs_completed = 0

    def do_cmd_list_props(self):
        print('Properties for', self.id)

        for cid, val in self.camera.properties.items():
            print('\t{}: {}'.format(cid, val))

    def do_cmd_list_controls(self):
        print('Controls for', self.id)

        for cid, info in self.camera.controls.items():
            print('\t{}: {}'.format(cid, info))

    def do_cmd_info(self):
        print('Stream info for', self.id)

        roles = [libcam.StreamRole.Viewfinder]

        camconfig = self.camera.generate_configuration(roles)
        if camconfig is None:
            raise Exception('Generating config failed')

        for i, stream_config in enumerate(camconfig):
            print('\t{}: {}'.format(i, stream_config))

            formats = stream_config.formats
            for fmt in formats.pixel_formats:
                print('\t * Pixelformat:', fmt, formats.range(fmt))

                for size in formats.sizes(fmt):
                    print('\t  -', size)

    def acquire(self):
        self.camera.acquire()

    def release(self):
        self.camera.release()

    def __parse_streams(self):
        streams = []

        for stream_desc in self.opt_stream:
            stream_opts: dict[str, Any]
            stream_opts = {'role': libcam.StreamRole.Viewfinder}

            for stream_opt in stream_desc.split(','):
                if stream_opt == 0:
                    continue

                arr = stream_opt.split('=')
                if len(arr) != 2:
                    print('Bad stream option', stream_opt)
                    sys.exit(-1)

                key = arr[0]
                value = arr[1]

                if key in ['width', 'height']:
                    value = int(value)
                elif key == 'role':
                    rolemap = {
                        'still': libcam.StreamRole.StillCapture,
                        'raw': libcam.StreamRole.Raw,
                        'video': libcam.StreamRole.VideoRecording,
                        'viewfinder': libcam.StreamRole.Viewfinder,
                    }

                    role = rolemap.get(value.lower(), None)

                    if role is None:
                        print('Bad stream role', value)
                        sys.exit(-1)

                    value = role
                elif key == 'pixelformat':
                    pass
                else:
                    print('Bad stream option key', key)
                    sys.exit(-1)

                stream_opts[key] = value

            streams.append(stream_opts)

        return streams

    def configure(self):
        streams = self.__parse_streams()

        roles = [opts['role'] for opts in streams]

        camconfig = self.camera.generate_configuration(roles)
        if camconfig is None:
            raise Exception('Generating config failed')

        for idx, stream_opts in enumerate(streams):
            stream_config = camconfig.at(idx)

            if 'width' in stream_opts:
                stream_config.size.width = stream_opts['width']

            if 'height' in stream_opts:
                stream_config.size.height = stream_opts['height']

            if 'pixelformat' in stream_opts:
                stream_config.pixel_format = libcam.PixelFormat(stream_opts['pixelformat'])

        stat = camconfig.validate()

        if stat == libcam.CameraConfiguration.Status.Invalid:
            print('Camera configuration invalid')
            exit(-1)
        elif stat == libcam.CameraConfiguration.Status.Adjusted:
            if self.opt_strict_formats:
                print('Adjusting camera configuration disallowed by --strict-formats argument')
                exit(-1)

            print('Camera configuration adjusted')

        r = self.camera.configure(camconfig)
        if r != 0:
            raise Exception('Configure failed')

        self.stream_names = {}
        self.streams = []

        for idx, stream_config in enumerate(camconfig):
            stream = stream_config.stream
            self.streams.append(stream)
            self.stream_names[stream] = 'stream' + str(idx)
            print('{}-{}: stream config {}'.format(self.id, self.stream_names[stream], stream.configuration))

    def alloc_buffers(self):
        allocator = libcam.FrameBufferAllocator(self.camera)

        for stream in self.streams:
            ret = allocator.allocate(stream)
            if ret < 0:
                print('Cannot allocate buffers')
                exit(-1)

            allocated = len(allocator.buffers(stream))

            print('{}-{}: Allocated {} buffers'.format(self.id, self.stream_names[stream], allocated))

        self.allocator = allocator

    def create_requests(self):
        self.requests = []

        # Identify the stream with the least number of buffers
        num_bufs = min([len(self.allocator.buffers(stream)) for stream in self.streams])

        requests = []

        for buf_num in range(num_bufs):
            request = self.camera.create_request(self.idx)

            if request is None:
                print('Can not create request')
                exit(-1)

            for stream in self.streams:
                buffers = self.allocator.buffers(stream)
                buffer = buffers[buf_num]

                ret = request.add_buffer(stream, buffer)
                if ret < 0:
                    print('Can not set buffer for request')
                    exit(-1)

            requests.append(request)

        self.requests = requests

    def start(self):
        self.camera.start()

    def stop(self):
        self.camera.stop()

    def queue_requests(self):
        for request in self.requests:
            self.camera.queue_request(request)
            self.reqs_queued += 1

        del self.requests


class CaptureState:
    cm: libcam.CameraManager
    contexts: list[CameraContext]
    renderer: Any

    def __init__(self, cm, contexts):
        self.cm = cm
        self.contexts = contexts

    # Called from renderer when there is a libcamera event
    def event_handler(self):
        try:
            reqs = self.cm.get_ready_requests()

            for req in reqs:
                ctx = next(ctx for ctx in self.contexts if ctx.idx == req.cookie)
                self.__request_handler(ctx, req)

            running = any(ctx.reqs_completed < ctx.opt_capture for ctx in self.contexts)
            return running
        except Exception:
            traceback.print_exc()
            return False

    def __request_handler(self, ctx, req):
        if req.status != libcam.Request.Status.Complete:
            raise Exception('{}: Request failed: {}'.format(ctx.id, req.status))

        buffers = req.buffers

        # Compute the frame rate. The timestamp is arbitrarily retrieved from
        # the first buffer, as all buffers should have matching timestamps.
        ts = buffers[next(iter(buffers))].metadata.timestamp
        last = ctx.last
        fps = 1000000000.0 / (ts - last) if (last != 0 and (ts - last) != 0) else 0
        ctx.last = ts
        ctx.fps = fps

        for stream, fb in buffers.items():
            stream_name = ctx.stream_names[stream]

            crcs = []
            if ctx.opt_crc:
                with libcamera.utils.MappedFrameBuffer(fb) as mfb:
                    plane_crcs = [binascii.crc32(p) for p in mfb.planes]
                    crcs.append(plane_crcs)

            meta = fb.metadata

            print('{:.6f} ({:.2f} fps) {}-{}: seq {}, bytes {}, CRCs {}'
                  .format(ts / 1000000000, fps,
                          ctx.id, stream_name,
                          meta.sequence,
                          '/'.join([str(p.bytes_used) for p in meta.planes]),
                          crcs))

            if ctx.opt_metadata:
                reqmeta = req.metadata
                for ctrl, val in reqmeta.items():
                    print(f'\t{ctrl} = {val}')

            if ctx.opt_save_frames:
                with libcamera.utils.MappedFrameBuffer(fb) as mfb:
                    filename = 'frame-{}-{}-{}.data'.format(ctx.id, stream_name, ctx.reqs_completed)
                    with open(filename, 'wb') as f:
                        for p in mfb.planes:
                            f.write(p)

        self.renderer.request_handler(ctx, req)

        ctx.reqs_completed += 1

    # Called from renderer when it has finished with a request
    def request_processed(self, ctx, req):
        if ctx.reqs_queued < ctx.opt_capture:
            req.reuse()
            ctx.camera.queue_request(req)
            ctx.reqs_queued += 1

    def __capture_init(self):
        for ctx in self.contexts:
            ctx.acquire()

        for ctx in self.contexts:
            ctx.configure()

        for ctx in self.contexts:
            ctx.alloc_buffers()

        for ctx in self.contexts:
            ctx.create_requests()

    def __capture_start(self):
        for ctx in self.contexts:
            ctx.start()

        for ctx in self.contexts:
            ctx.queue_requests()

    def __capture_deinit(self):
        for ctx in self.contexts:
            ctx.stop()

        for ctx in self.contexts:
            ctx.release()

    def do_cmd_capture(self):
        self.__capture_init()

        self.renderer.setup()

        self.__capture_start()

        self.renderer.run()

        self.__capture_deinit()


class CustomAction(argparse.Action):
    def __init__(self, option_strings, dest, **kwargs):
        super().__init__(option_strings, dest, default={}, **kwargs)

    def __call__(self, parser, namespace, values, option_string=None):
        if len(namespace.camera) == 0:
            print(f'Option {option_string} requires a --camera context')
            sys.exit(-1)

        if self.type == bool:
            values = True

        current = namespace.camera[-1]

        data = getattr(namespace, self.dest)

        if self.nargs == '+':
            if current not in data:
                data[current] = []

            data[current] += values
        else:
            data[current] = values


def do_cmd_list(cm):
    print('Available cameras:')

    for idx, c in enumerate(cm.cameras):
        print(f'{idx + 1}: {c.id}')


def main():
    parser = argparse.ArgumentParser()
    # global options
    parser.add_argument('-l', '--list', action='store_true', help='List all cameras')
    parser.add_argument('-c', '--camera', type=int, action='extend', nargs=1, default=[], help='Specify which camera to operate on, by index')
    parser.add_argument('-p', '--list-properties', action='store_true', help='List cameras properties')
    parser.add_argument('--list-controls', action='store_true', help='List cameras controls')
    parser.add_argument('-I', '--info', action='store_true', help='Display information about stream(s)')
    parser.add_argument('-R', '--renderer', default='null', help='Renderer (null, kms, qt, qtgl)')

    # per camera options
    parser.add_argument('-C', '--capture', nargs='?', type=int, const=1000000, action=CustomAction, help='Capture until interrupted by user or until CAPTURE frames captured')
    parser.add_argument('--crc', nargs=0, type=bool, action=CustomAction, help='Print CRC32 for captured frames')
    parser.add_argument('--save-frames', nargs=0, type=bool, action=CustomAction, help='Save captured frames to files')
    parser.add_argument('--metadata', nargs=0, type=bool, action=CustomAction, help='Print the metadata for completed requests')
    parser.add_argument('--strict-formats', type=bool, nargs=0, action=CustomAction, help='Do not allow requested stream format(s) to be adjusted')
    parser.add_argument('-s', '--stream', nargs='+', action=CustomAction)
    args = parser.parse_args()

    cm = libcam.CameraManager.singleton()

    if args.list:
        do_cmd_list(cm)

    contexts = []

    for cam_idx in args.camera:
        camera = next((c for i, c in enumerate(cm.cameras) if i + 1 == cam_idx), None)

        if camera is None:
            print('Unable to find camera', cam_idx)
            return -1

        ctx = CameraContext(camera, cam_idx)
        ctx.opt_capture = args.capture.get(cam_idx, 0)
        ctx.opt_crc = args.crc.get(cam_idx, False)
        ctx.opt_save_frames = args.save_frames.get(cam_idx, False)
        ctx.opt_metadata = args.metadata.get(cam_idx, False)
        ctx.opt_strict_formats = args.strict_formats.get(cam_idx, False)
        ctx.opt_stream = args.stream.get(cam_idx, ['role=viewfinder'])
        contexts.append(ctx)

    for ctx in contexts:
        print('Using camera {} as {}'.format(ctx.camera.id, ctx.id))

    for ctx in contexts:
        if args.list_properties:
            ctx.do_cmd_list_props()
        if args.list_controls:
            ctx.do_cmd_list_controls()
        if args.info:
            ctx.do_cmd_info()

    # Filter out capture contexts which are not marked for capture
    contexts = [ctx for ctx in contexts if ctx.opt_capture > 0]

    if contexts:
        state = CaptureState(cm, contexts)

        if args.renderer == 'null':
            import cam_null
            renderer = cam_null.NullRenderer(state)
        elif args.renderer == 'kms':
            import cam_kms
            renderer = cam_kms.KMSRenderer(state)
        elif args.renderer == 'qt':
            import cam_qt
            renderer = cam_qt.QtRenderer(state)
        elif args.renderer == 'qtgl':
            import cam_qtgl
            renderer = cam_qtgl.QtRenderer(state)
        else:
            print('Bad renderer', args.renderer)
            return -1

        state.renderer = renderer

        state.do_cmd_capture()

    return 0


if __name__ == '__main__':
    sys.exit(main())
