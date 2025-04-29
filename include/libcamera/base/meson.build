# SPDX-License-Identifier: CC0-1.0

libcamera_base_include_dir = libcamera_include_dir / 'base'

libcamera_base_public_headers = files([
    'bound_method.h',
    'class.h',
    'flags.h',
    'object.h',
    'shared_fd.h',
    'signal.h',
    'span.h',
    'unique_fd.h',
])

libcamera_base_private_headers = files([
    'backtrace.h',
    'event_dispatcher.h',
    'event_dispatcher_poll.h',
    'event_notifier.h',
    'file.h',
    'log.h',
    'memfd.h',
    'message.h',
    'mutex.h',
    'private.h',
    'semaphore.h',
    'thread.h',
    'thread_annotations.h',
    'timer.h',
    'utils.h',
])

libcamera_base_headers = [
    libcamera_base_public_headers,
    libcamera_base_private_headers,
]

install_headers(libcamera_base_public_headers,
                subdir : libcamera_base_include_dir)
