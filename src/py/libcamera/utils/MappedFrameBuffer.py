# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

import libcamera
from typing import Tuple

class MappedFrameBuffer:
    """
    Provides memoryviews for the FrameBuffer's planes
    """
    def __init__(self, fb: libcamera.FrameBuffer):
        self.__fb = fb

    def __enter__(self):
        import os
        import mmap

        fb = self.__fb

        # Collect information about the buffers

        bufinfos = {}

        for plane in fb.planes:
            fd = plane.fd

            if fd not in bufinfos:
                buflen = os.lseek(fd, 0, os.SEEK_END)
                bufinfos[fd] = {'maplen': 0, 'buflen': buflen}
            else:
                buflen = bufinfos[fd]['buflen']

            if plane.offset > buflen or plane.offset + plane.length > buflen:
                raise RuntimeError(f'plane is out of buffer: buffer length={buflen}, ' +
                                   f'plane offset={plane.offset}, plane length={plane.length}')

            bufinfos[fd]['maplen'] = max(bufinfos[fd]['maplen'], plane.offset + plane.length)

        # mmap the buffers

        maps = []

        for fd, info in bufinfos.items():
            map = mmap.mmap(fd, info['maplen'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            info['map'] = map
            maps.append(map)

        self.__maps = tuple(maps)

        # Create memoryviews for the planes

        planes = []

        for plane in fb.planes:
            fd = plane.fd
            info = bufinfos[fd]

            mv = memoryview(info['map'])

            start = plane.offset
            end = plane.offset + plane.length

            mv = mv[start:end]

            planes.append(mv)

        self.__planes = tuple(planes)

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        for p in self.__planes:
            p.release()

        for mm in self.__maps:
            mm.close()

    @property
    def planes(self) -> Tuple[memoryview, ...]:
        """memoryviews for the planes"""
        return self.__planes
