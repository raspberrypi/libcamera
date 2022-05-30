# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

import pykms
import selectors
import sys


class KMSRenderer:
    def __init__(self, state):
        self.state = state

        self.cm = state.cm
        self.contexts = state.contexts
        self.running = False

        card = pykms.Card()

        res = pykms.ResourceManager(card)
        conn = res.reserve_connector()
        crtc = res.reserve_crtc(conn)
        mode = conn.get_default_mode()
        modeb = mode.to_blob(card)

        req = pykms.AtomicReq(card)
        req.add_connector(conn, crtc)
        req.add_crtc(crtc, modeb)
        r = req.commit_sync(allow_modeset=True)
        assert(r == 0)

        self.card = card
        self.resman = res
        self.crtc = crtc
        self.mode = mode

        self.bufqueue = []
        self.current = None
        self.next = None
        self.cam_2_drm = {}

    # KMS

    def close(self):
        req = pykms.AtomicReq(self.card)
        for s in self.streams:
            req.add_plane(s['plane'], None, None, dst=(0, 0, 0, 0))
        req.commit()

    def add_plane(self, req, stream, fb):
        s = next(s for s in self.streams if s['stream'] == stream)
        idx = s['idx']
        plane = s['plane']

        if idx % 2 == 0:
            x = 0
        else:
            x = self.mode.hdisplay - fb.width

        if idx // 2 == 0:
            y = 0
        else:
            y = self.mode.vdisplay - fb.height

        req.add_plane(plane, fb, self.crtc, dst=(x, y, fb.width, fb.height))

    def apply_request(self, drmreq):

        buffers = drmreq['camreq'].buffers

        req = pykms.AtomicReq(self.card)

        for stream, fb in buffers.items():
            drmfb = self.cam_2_drm.get(fb, None)
            self.add_plane(req, stream, drmfb)

        req.commit()

    def handle_page_flip(self, frame, time):
        old = self.current
        self.current = self.next

        if len(self.bufqueue) > 0:
            self.next = self.bufqueue.pop(0)
        else:
            self.next = None

        if self.next:
            drmreq = self.next

            self.apply_request(drmreq)

        if old:
            req = old['camreq']
            ctx = old['camctx']
            self.state.request_processed(ctx, req)

    def queue(self, drmreq):
        if not self.next:
            self.next = drmreq
            self.apply_request(drmreq)
        else:
            self.bufqueue.append(drmreq)

    # libcamera

    def setup(self):
        self.streams = []

        idx = 0
        for ctx in self.contexts:
            for stream in ctx.streams:

                cfg = stream.configuration
                fmt = cfg.pixel_format
                fmt = pykms.PixelFormat(fmt.fourcc)

                plane = self.resman.reserve_generic_plane(self.crtc, fmt)
                assert(plane is not None)

                self.streams.append({
                    'idx': idx,
                    'stream': stream,
                    'plane': plane,
                    'fmt': fmt,
                    'size': cfg.size,
                })

                for fb in ctx.allocator.buffers(stream):
                    w = cfg.size.width
                    h = cfg.size.height
                    fds = []
                    strides = []
                    offsets = []
                    for plane in fb.planes:
                        fds.append(plane.fd)
                        strides.append(cfg.stride)
                        offsets.append(plane.offset)

                    drmfb = pykms.DmabufFramebuffer(self.card, w, h, fmt,
                                                    fds, strides, offsets)
                    self.cam_2_drm[fb] = drmfb

                idx += 1

    def readdrm(self, fileobj):
        for ev in self.card.read_events():
            if ev.type == pykms.DrmEventType.FLIP_COMPLETE:
                self.handle_page_flip(ev.seq, ev.time)

    def readcam(self, fd):
        self.running = self.state.event_handler()

    def readkey(self, fileobj):
        sys.stdin.readline()
        self.running = False

    def run(self):
        print('Capturing...')

        self.running = True

        sel = selectors.DefaultSelector()
        sel.register(self.card.fd, selectors.EVENT_READ, self.readdrm)
        sel.register(self.cm.event_fd, selectors.EVENT_READ, self.readcam)
        sel.register(sys.stdin, selectors.EVENT_READ, self.readkey)

        print('Press enter to exit')

        while self.running:
            events = sel.select()
            for key, mask in events:
                callback = key.data
                callback(key.fileobj)

        print('Exiting...')

    def request_handler(self, ctx, req):

        drmreq = {
            'camctx': ctx,
            'camreq': req,
        }

        self.queue(drmreq)
