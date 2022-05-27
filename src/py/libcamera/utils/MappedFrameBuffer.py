# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

class MappedFrameBuffer:
    def __init__(self, fb):
        self.__fb = fb

    def __enter__(self):
        import os
        import mmap

        fb = self.__fb

        # Collect information about the buffers

        bufinfos = {}

        for i in range(fb.num_planes):
            fd = fb.fd(i)

            if fd not in bufinfos:
                buflen = os.lseek(fd, 0, os.SEEK_END)
                bufinfos[fd] = {'maplen': 0, 'buflen': buflen}
            else:
                buflen = bufinfos[fd]['buflen']

            if fb.offset(i) > buflen or fb.offset(i) + fb.length(i) > buflen:
                raise RuntimeError(f'plane is out of buffer: buffer length={buflen}, ' +
                                   f'plane offset={fb.offset(i)}, plane length={fb.length(i)}')

            bufinfos[fd]['maplen'] = max(bufinfos[fd]['maplen'], fb.offset(i) + fb.length(i))

        # mmap the buffers

        maps = []

        for fd, info in bufinfos.items():
            map = mmap.mmap(fd, info['maplen'], mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
            info['map'] = map
            maps.append(map)

        self.__maps = tuple(maps)

        # Create memoryviews for the planes

        planes = []

        for i in range(fb.num_planes):
            fd = fb.fd(i)
            info = bufinfos[fd]

            mv = memoryview(info['map'])

            start = fb.offset(i)
            end = fb.offset(i) + fb.length(i)

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
    def planes(self):
        return self.__planes
