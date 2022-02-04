# SPDX-License-Identifier: LGPL-2.1-or-later
# Copyright (C) 2021, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

from ._libcamera import *
import mmap

def __FrameBuffer__mmap(self, plane):
	total_length = sum(self.metadata.bytesused)
	return mmap.mmap(self.fd(plane), total_length, mmap.MAP_SHARED, mmap.PROT_WRITE)

FrameBuffer.mmap = __FrameBuffer__mmap
