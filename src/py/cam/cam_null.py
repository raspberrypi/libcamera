# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

import selectors
import sys


class NullRenderer:
    def __init__(self, state):
        self.state = state

        self.cm = state.cm
        self.contexts = state.contexts

        self.running = False

    def setup(self):
        pass

    def run(self):
        print('Capturing...')

        self.running = True

        sel = selectors.DefaultSelector()
        sel.register(self.cm.event_fd, selectors.EVENT_READ, self.readcam)
        sel.register(sys.stdin, selectors.EVENT_READ, self.readkey)

        print('Press enter to exit')

        while self.running:
            events = sel.select()
            for key, mask in events:
                callback = key.data
                callback(key.fileobj)

        print('Exiting...')

    def readcam(self, fd):
        self.running = self.state.event_handler()

    def readkey(self, fileobj):
        sys.stdin.readline()
        self.running = False

    def request_handler(self, ctx, req):
        self.state.request_processed(ctx, req)
