#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from collections import defaultdict
import gc
import libcamera as libcam
import selectors
import typing
import unittest
import weakref


class BaseTestCase(unittest.TestCase):
    def assertZero(self, a, msg=None):
        self.assertEqual(a, 0, msg)

    def assertIsAlive(self, wr, msg='object not alive'):
        self.assertIsNotNone(wr(), msg)

    def assertIsDead(self, wr, msg='object not dead'):
        self.assertIsNone(wr(), msg)

    def assertIsAllAlive(self, wr_list, msg='object not alive'):
        self.assertTrue(all([wr() for wr in wr_list]), msg)

    def assertIsAllDead(self, wr_list, msg='object not dead'):
        self.assertTrue(all([not wr() for wr in wr_list]), msg)


class SimpleTestMethods(BaseTestCase):
    def test_get_ref(self):
        cm = libcam.CameraManager.singleton()
        wr_cm = weakref.ref(cm)

        cam = cm.get('platform/vimc.0 Sensor B')
        self.assertIsNotNone(cam)
        wr_cam = weakref.ref(cam)

        del cm
        gc.collect()
        self.assertIsAlive(wr_cm)

        del cam
        gc.collect()
        self.assertIsDead(wr_cm)
        self.assertIsDead(wr_cam)

    def test_acquire_release(self):
        cm = libcam.CameraManager.singleton()
        cam = cm.get('platform/vimc.0 Sensor B')
        self.assertIsNotNone(cam)

        cam.acquire()

        cam.release()

    def test_double_acquire(self):
        cm = libcam.CameraManager.singleton()
        cam = cm.get('platform/vimc.0 Sensor B')
        self.assertIsNotNone(cam)

        cam.acquire()

        libcam.log_set_level('Camera', 'FATAL')
        with self.assertRaises(RuntimeError):
            cam.acquire()
        libcam.log_set_level('Camera', 'INFO')

        cam.release()

        # I expected exception here, but looks like double release works fine
        cam.release()

    def test_version(self):
        cm = libcam.CameraManager.singleton()
        self.assertIsInstance(cm.version, str)


class CameraTesterBase(BaseTestCase):
    cm: typing.Any
    cam: typing.Any

    def setUp(self):
        self.cm = libcam.CameraManager.singleton()
        self.cam = next((cam for cam in self.cm.cameras if 'platform/vimc' in cam.id), None)
        if self.cam is None:
            self.cm = None
            self.skipTest('No vimc found')

        self.cam.acquire()

        self.wr_cam = weakref.ref(self.cam)
        self.wr_cm = weakref.ref(self.cm)

    def tearDown(self):
        # If a test fails, the camera may be in running state. So always stop.
        self.cam.stop()

        self.cam.release()

        self.cam = None
        self.cm = None

        self.assertIsDead(self.wr_cm)
        self.assertIsDead(self.wr_cam)


class AllocatorTestMethods(CameraTesterBase):
    def test_allocator(self):
        cam = self.cam

        camconfig = cam.generate_configuration([libcam.StreamRole.StillCapture])
        self.assertTrue(camconfig.size == 1)
        wr_camconfig = weakref.ref(camconfig)

        streamconfig = camconfig.at(0)
        wr_streamconfig = weakref.ref(streamconfig)

        cam.configure(camconfig)

        stream = streamconfig.stream
        wr_stream = weakref.ref(stream)

        # stream should keep streamconfig and camconfig alive
        del streamconfig
        del camconfig
        gc.collect()
        self.assertIsAlive(wr_camconfig)
        self.assertIsAlive(wr_streamconfig)

        allocator = libcam.FrameBufferAllocator(cam)
        num_bufs = allocator.allocate(stream)
        self.assertTrue(num_bufs > 0)
        wr_allocator = weakref.ref(allocator)

        buffers = allocator.buffers(stream)
        self.assertIsNotNone(buffers)
        del buffers

        buffer = allocator.buffers(stream)[0]
        self.assertIsNotNone(buffer)
        wr_buffer = weakref.ref(buffer)

        del allocator
        gc.collect()
        self.assertIsAlive(wr_buffer)
        self.assertIsAlive(wr_allocator)
        self.assertIsAlive(wr_stream)

        del buffer
        gc.collect()
        self.assertIsDead(wr_buffer)
        self.assertIsDead(wr_allocator)
        self.assertIsAlive(wr_stream)

        del stream
        gc.collect()
        self.assertIsDead(wr_stream)
        self.assertIsDead(wr_camconfig)
        self.assertIsDead(wr_streamconfig)


class SimpleCaptureMethods(CameraTesterBase):
    def test_blocking(self):
        cm = self.cm
        cam = self.cam

        camconfig = cam.generate_configuration([libcam.StreamRole.StillCapture])
        self.assertTrue(camconfig.size == 1)

        streamconfig = camconfig.at(0)
        fmts = streamconfig.formats
        self.assertIsNotNone(fmts)
        fmts = None

        cam.configure(camconfig)

        stream = streamconfig.stream

        allocator = libcam.FrameBufferAllocator(cam)
        num_bufs = allocator.allocate(stream)
        self.assertTrue(num_bufs > 0)

        num_bufs = len(allocator.buffers(stream))

        reqs = []
        for i in range(num_bufs):
            req = cam.create_request(i)
            self.assertIsNotNone(req)

            buffer = allocator.buffers(stream)[i]
            req.add_buffer(stream, buffer)

            reqs.append(req)

        buffer = None

        cam.start()

        for req in reqs:
            cam.queue_request(req)

        reqs = None
        gc.collect()

        sel = selectors.DefaultSelector()
        sel.register(cm.event_fd, selectors.EVENT_READ)

        reqs = []

        while True:
            events = sel.select()
            if not events:
                continue

            ready_reqs = cm.get_ready_requests()

            reqs += ready_reqs

            if len(reqs) == num_bufs:
                break

        for i, req in enumerate(reqs):
            self.assertTrue(i == req.cookie)

        reqs = None
        gc.collect()

        cam.stop()

    def test_select(self):
        cm = self.cm
        cam = self.cam

        camconfig = cam.generate_configuration([libcam.StreamRole.StillCapture])
        self.assertTrue(camconfig.size == 1)

        streamconfig = camconfig.at(0)
        fmts = streamconfig.formats
        self.assertIsNotNone(fmts)
        fmts = None

        cam.configure(camconfig)

        stream = streamconfig.stream

        allocator = libcam.FrameBufferAllocator(cam)
        num_bufs = allocator.allocate(stream)
        self.assertTrue(num_bufs > 0)

        num_bufs = len(allocator.buffers(stream))

        reqs = []
        for i in range(num_bufs):
            req = cam.create_request(i)
            self.assertIsNotNone(req)

            buffer = allocator.buffers(stream)[i]
            req.add_buffer(stream, buffer)

            reqs.append(req)

        buffer = None

        cam.start()

        for req in reqs:
            cam.queue_request(req)

        reqs = None
        gc.collect()

        sel = selectors.DefaultSelector()
        sel.register(cm.event_fd, selectors.EVENT_READ)

        reqs = []

        running = True
        while running:
            events = sel.select()
            for _ in events:
                ready_reqs = cm.get_ready_requests()

                reqs += ready_reqs

                if len(reqs) == num_bufs:
                    running = False

        self.assertTrue(len(reqs) == num_bufs)

        for i, req in enumerate(reqs):
            self.assertTrue(i == req.cookie)

        reqs = None
        gc.collect()

        cam.stop()


# Recursively expand slist's objects into olist, using seen to track already
# processed objects.
def _getr(slist, olist, seen):
    for e in slist:
        if id(e) in seen:
            continue
        seen.add(id(e))
        olist.append(e)
        tl = gc.get_referents(e)
        if tl:
            _getr(tl, olist, seen)


def get_all_objects(ignored=[]):
    gcl = gc.get_objects()
    olist = []
    seen = set()

    seen.add(id(gcl))
    seen.add(id(olist))
    seen.add(id(seen))
    seen.update(set([id(o) for o in ignored]))

    _getr(gcl, olist, seen)

    return olist


def create_type_count_map(olist):
    map = defaultdict(int)
    for o in olist:
        map[type(o)] += 1
    return map


def diff_type_count_maps(before, after):
    return [(k, after[k] - before[k]) for k in after if after[k] != before[k]]


if __name__ == '__main__':
    # \todo This is an attempt to see the Python objects that are not collected,
    # but this doesn't work very well, as things always leak a bit.
    test_leaks = False

    if test_leaks:
        gc.unfreeze()
        gc.collect()

        obs_before = get_all_objects()

    unittest.main(exit=False)

    if test_leaks:
        gc.unfreeze()
        gc.collect()

        obs_after = get_all_objects([obs_before])   # type: ignore

        before = create_type_count_map(obs_before)  # type: ignore
        after = create_type_count_map(obs_after)

        leaks = diff_type_count_maps(before, after)
        if len(leaks) > 0:
            print(leaks)
