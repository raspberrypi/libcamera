# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2021, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from ._libcamera import *
import mmap

def __FrameBuffer__mmap(self, plane):
	return mmap.mmap(self.fd(plane), self.length(plane), mmap.MAP_SHARED, mmap.PROT_WRITE)

FrameBuffer.mmap = __FrameBuffer__mmap
