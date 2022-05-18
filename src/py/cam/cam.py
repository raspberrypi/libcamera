#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

# \todo Convert ctx and state dicts to proper classes, and move relevant
#       functions to those classes.

import argparse
import binascii
import libcamera as libcam
import os
import sys
import traceback


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


def do_cmd_list_props(ctx):
    camera = ctx['camera']

    print('Properties for', ctx['id'])

    for name, prop in camera.properties.items():
        print('\t{}: {}'.format(name, prop))


def do_cmd_list_controls(ctx):
    camera = ctx['camera']

    print('Controls for', ctx['id'])

    for name, prop in camera.controls.items():
        print('\t{}: {}'.format(name, prop))


def do_cmd_info(ctx):
    camera = ctx['camera']

    print('Stream info for', ctx['id'])

    roles = [libcam.StreamRole.Viewfinder]

    camconfig = camera.generate_configuration(roles)
    if camconfig is None:
        raise Exception('Generating config failed')

    for i, stream_config in enumerate(camconfig):
        print('\t{}: {}'.format(i, stream_config))

        formats = stream_config.formats
        for fmt in formats.pixel_formats:
            print('\t * Pixelformat:', fmt, formats.range(fmt))

            for size in formats.sizes(fmt):
                print('\t  -', size)


def acquire(ctx):
    camera = ctx['camera']

    camera.acquire()


def release(ctx):
    camera = ctx['camera']

    camera.release()


def parse_streams(ctx):
    streams = []

    for stream_desc in ctx['opt-stream']:
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


def configure(ctx):
    camera = ctx['camera']

    streams = parse_streams(ctx)

    roles = [opts['role'] for opts in streams]

    camconfig = camera.generate_configuration(roles)
    if camconfig is None:
        raise Exception('Generating config failed')

    for idx, stream_opts in enumerate(streams):
        stream_config = camconfig.at(idx)

        if 'width' in stream_opts and 'height' in stream_opts:
            stream_config.size = (stream_opts['width'], stream_opts['height'])

        if 'pixelformat' in stream_opts:
            stream_config.pixel_format = libcam.PixelFormat(stream_opts['pixelformat'])

    stat = camconfig.validate()

    if stat == libcam.CameraConfiguration.Status.Invalid:
        print('Camera configuration invalid')
        exit(-1)
    elif stat == libcam.CameraConfiguration.Status.Adjusted:
        if ctx['opt-strict-formats']:
            print('Adjusting camera configuration disallowed by --strict-formats argument')
            exit(-1)

        print('Camera configuration adjusted')

    r = camera.configure(camconfig)
    if r != 0:
        raise Exception('Configure failed')

    ctx['stream-names'] = {}
    ctx['streams'] = []

    for idx, stream_config in enumerate(camconfig):
        stream = stream_config.stream
        ctx['streams'].append(stream)
        ctx['stream-names'][stream] = 'stream' + str(idx)
        print('{}-{}: stream config {}'.format(ctx['id'], ctx['stream-names'][stream], stream.configuration))


def alloc_buffers(ctx):
    camera = ctx['camera']

    allocator = libcam.FrameBufferAllocator(camera)

    for idx, stream in enumerate(ctx['streams']):
        ret = allocator.allocate(stream)
        if ret < 0:
            print('Cannot allocate buffers')
            exit(-1)

        allocated = len(allocator.buffers(stream))

        print('{}-{}: Allocated {} buffers'.format(ctx['id'], ctx['stream-names'][stream], allocated))

    ctx['allocator'] = allocator


def create_requests(ctx):
    camera = ctx['camera']

    ctx['requests'] = []

    # Identify the stream with the least number of buffers
    num_bufs = min([len(ctx['allocator'].buffers(stream)) for stream in ctx['streams']])

    requests = []

    for buf_num in range(num_bufs):
        request = camera.create_request(ctx['idx'])

        if request is None:
            print('Can not create request')
            exit(-1)

        for stream in ctx['streams']:
            buffers = ctx['allocator'].buffers(stream)
            buffer = buffers[buf_num]

            ret = request.add_buffer(stream, buffer)
            if ret < 0:
                print('Can not set buffer for request')
                exit(-1)

        requests.append(request)

    ctx['requests'] = requests


def start(ctx):
    camera = ctx['camera']

    camera.start()


def stop(ctx):
    camera = ctx['camera']

    camera.stop()


def queue_requests(ctx):
    camera = ctx['camera']

    for request in ctx['requests']:
        camera.queue_request(request)
        ctx['reqs-queued'] += 1

    del ctx['requests']


def capture_init(contexts):
    for ctx in contexts:
        acquire(ctx)

    for ctx in contexts:
        configure(ctx)

    for ctx in contexts:
        alloc_buffers(ctx)

    for ctx in contexts:
        create_requests(ctx)


def capture_start(contexts):
    for ctx in contexts:
        start(ctx)

    for ctx in contexts:
        queue_requests(ctx)


# Called from renderer when there is a libcamera event
def event_handler(state):
    try:
        cm = state['cm']
        contexts = state['contexts']

        os.read(cm.efd, 8)

        reqs = cm.get_ready_requests()

        for req in reqs:
            ctx = next(ctx for ctx in contexts if ctx['idx'] == req.cookie)
            request_handler(state, ctx, req)

        running = any(ctx['reqs-completed'] < ctx['opt-capture'] for ctx in contexts)
        return running
    except Exception as e:
        traceback.print_exc()
        return False


def request_handler(state, ctx, req):
    if req.status != libcam.Request.Status.Complete:
        raise Exception('{}: Request failed: {}'.format(ctx['id'], req.status))

    buffers = req.buffers

    # Compute the frame rate. The timestamp is arbitrarily retrieved from
    # the first buffer, as all buffers should have matching timestamps.
    ts = buffers[next(iter(buffers))].metadata.timestamp
    last = ctx.get('last', 0)
    fps = 1000000000.0 / (ts - last) if (last != 0 and (ts - last) != 0) else 0
    ctx['last'] = ts
    ctx['fps'] = fps

    for stream, fb in buffers.items():
        stream_name = ctx['stream-names'][stream]

        crcs = []
        if ctx['opt-crc']:
            with fb.mmap() as mfb:
                plane_crcs = [binascii.crc32(p) for p in mfb.planes]
                crcs.append(plane_crcs)

        meta = fb.metadata

        print('{:.6f} ({:.2f} fps) {}-{}: seq {}, bytes {}, CRCs {}'
              .format(ts / 1000000000, fps,
                      ctx['id'], stream_name,
                      meta.sequence, meta.bytesused,
                      crcs))

        if ctx['opt-metadata']:
            reqmeta = req.metadata
            for ctrl, val in reqmeta.items():
                print(f'\t{ctrl} = {val}')

        if ctx['opt-save-frames']:
            with fb.mmap() as mfb:
                filename = 'frame-{}-{}-{}.data'.format(ctx['id'], stream_name, ctx['reqs-completed'])
                with open(filename, 'wb') as f:
                    for p in mfb.planes:
                        f.write(p)

    state['renderer'].request_handler(ctx, req)

    ctx['reqs-completed'] += 1


# Called from renderer when it has finished with a request
def request_prcessed(ctx, req):
    camera = ctx['camera']

    if ctx['reqs-queued'] < ctx['opt-capture']:
        req.reuse()
        camera.queue_request(req)
        ctx['reqs-queued'] += 1


def capture_deinit(contexts):
    for ctx in contexts:
        stop(ctx)

    for ctx in contexts:
        release(ctx)


def do_cmd_capture(state):
    capture_init(state['contexts'])

    renderer = state['renderer']

    renderer.setup()

    capture_start(state['contexts'])

    renderer.run()

    capture_deinit(state['contexts'])


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

        contexts.append({
                        'camera': camera,
                        'idx': cam_idx,
                        'id': 'cam' + str(cam_idx),
                        'reqs-queued': 0,
                        'reqs-completed': 0,
                        'opt-capture': args.capture.get(cam_idx, False),
                        'opt-crc': args.crc.get(cam_idx, False),
                        'opt-save-frames': args.save_frames.get(cam_idx, False),
                        'opt-metadata': args.metadata.get(cam_idx, False),
                        'opt-strict-formats': args.strict_formats.get(cam_idx, False),
                        'opt-stream': args.stream.get(cam_idx, ['role=viewfinder']),
                        })

    for ctx in contexts:
        print('Using camera {} as {}'.format(ctx['camera'].id, ctx['id']))

    for ctx in contexts:
        if args.list_properties:
            do_cmd_list_props(ctx)
        if args.list_controls:
            do_cmd_list_controls(ctx)
        if args.info:
            do_cmd_info(ctx)

    if args.capture:

        state = {
            'cm': cm,
            'contexts': contexts,
            'event_handler': event_handler,
            'request_prcessed': request_prcessed,
        }

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

        state['renderer'] = renderer

        do_cmd_capture(state)

    return 0


if __name__ == '__main__':
    sys.exit(main())
