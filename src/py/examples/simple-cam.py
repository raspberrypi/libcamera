#!/usr/bin/env python3

# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2022, Tomi Valkeinen <tomi.valkeinen@ideasonboard.com>

# A simple libcamera capture example
#
# This is a python version of simple-cam from:
# https://git.libcamera.org/libcamera/simple-cam.git
#
# \todo Move to simple-cam repository when the Python API has stabilized more

import libcamera as libcam
import selectors
import sys
import time

TIMEOUT_SEC = 3


def handle_camera_event(cm):
    # cm.get_ready_requests() returns the ready requests, which in our case
    # should almost always return a single Request, but in some cases there
    # could be multiple or none.

    reqs = cm.get_ready_requests()

    # Process the captured frames

    for req in reqs:
        process_request(req)


def process_request(request):
    global camera

    print()

    print(f'Request completed: {request}')

    # When a request has completed, it is populated with a metadata control
    # list that allows an application to determine various properties of
    # the completed request. This can include the timestamp of the Sensor
    # capture, or its gain and exposure values, or properties from the IPA
    # such as the state of the 3A algorithms.
    #
    # To examine each request, print all the metadata for inspection. A custom
    # application can parse each of these items and process them according to
    # its needs.

    requestMetadata = request.metadata
    for id, value in requestMetadata.items():
        print(f'\t{id.name} = {value}')

    # Each buffer has its own FrameMetadata to describe its state, or the
    # usage of each buffer. While in our simple capture we only provide one
    # buffer per request, a request can have a buffer for each stream that
    # is established when configuring the camera.
    #
    # This allows a viewfinder and a still image to be processed at the
    # same time, or to allow obtaining the RAW capture buffer from the
    # sensor along with the image as processed by the ISP.

    buffers = request.buffers
    for _, buffer in buffers.items():
        metadata = buffer.metadata

        # Print some information about the buffer which has completed.
        print(f' seq: {metadata.sequence:06} timestamp: {metadata.timestamp} bytesused: ' +
              '/'.join([str(p.bytes_used) for p in metadata.planes]))

        # Image data can be accessed here, but the FrameBuffer
        # must be mapped by the application

    # Re-queue the Request to the camera.
    request.reuse()
    camera.queue_request(request)


# ----------------------------------------------------------------------------
# Camera Naming.
#
# Applications are responsible for deciding how to name cameras, and present
# that information to the users. Every camera has a unique identifier, though
# this string is not designed to be friendly for a human reader.
#
# To support human consumable names, libcamera provides camera properties
# that allow an application to determine a naming scheme based on its needs.
#
# In this example, we focus on the location property, but also detail the
# model string for external cameras, as this is more likely to be visible
# information to the user of an externally connected device.
#
# The unique camera ID is appended for informative purposes.
#
def camera_name(camera):
    props = camera.properties
    location = props.get(libcam.properties.Location, None)

    if location == libcam.properties.LocationEnum.Front:
        name = 'Internal front camera'
    elif location == libcam.properties.LocationEnum.Back:
        name = 'Internal back camera'
    elif location == libcam.properties.LocationEnum.External:
        name = 'External camera'
        if libcam.properties.Model in props:
            name += f' "{props[libcam.properties.Model]}"'
    else:
        name = 'Undefined location'

    name += f' ({camera.id})'

    return name


def main():
    global camera

    # --------------------------------------------------------------------
    # Get the Camera Manager.
    #
    # The Camera Manager is responsible for enumerating all the Camera
    # in the system, by associating Pipeline Handlers with media entities
    # registered in the system.
    #
    # The CameraManager provides a list of available Cameras that
    # applications can operate on.
    #
    # There can only be a single CameraManager within any process space.

    cm = libcam.CameraManager.singleton()

    # Just as a test, generate names of the Cameras registered in the
    # system, and list them.

    for camera in cm.cameras:
        print(f' - {camera_name(camera)}')

    # --------------------------------------------------------------------
    # Camera
    #
    # Camera are entities created by pipeline handlers, inspecting the
    # entities registered in the system and reported to applications
    # by the CameraManager.
    #
    # In general terms, a Camera corresponds to a single image source
    # available in the system, such as an image sensor.
    #
    # Application lock usage of Camera by 'acquiring' them.
    # Once done with it, application shall similarly 'release' the Camera.
    #
    # As an example, use the first available camera in the system after
    # making sure that at least one camera is available.
    #
    # Cameras can be obtained by their ID or their index, to demonstrate
    # this, the following code gets the ID of the first camera; then gets
    # the camera associated with that ID (which is of course the same as
    # cm.cameras[0]).

    if not cm.cameras:
        print('No cameras were identified on the system.')
        return -1

    camera_id = cm.cameras[0].id
    camera = cm.get(camera_id)
    camera.acquire()

    # --------------------------------------------------------------------
    # Stream
    #
    # Each Camera supports a variable number of Stream. A Stream is
    # produced by processing data produced by an image source, usually
    # by an ISP.
    #
    #   +-------------------------------------------------------+
    #   | Camera                                                |
    #   |                +-----------+                          |
    #   | +--------+     |           |------> [  Main output  ] |
    #   | | Image  |     |           |                          |
    #   | |        |---->|    ISP    |------> [   Viewfinder  ] |
    #   | | Source |     |           |                          |
    #   | +--------+     |           |------> [ Still Capture ] |
    #   |                +-----------+                          |
    #   +-------------------------------------------------------+
    #
    # The number and capabilities of the Stream in a Camera are
    # a platform dependent property, and it's the pipeline handler
    # implementation that has the responsibility of correctly
    # report them.

    # --------------------------------------------------------------------
    # Camera Configuration.
    #
    # Camera configuration is tricky! It boils down to assign resources
    # of the system (such as DMA engines, scalers, format converters) to
    # the different image streams an application has requested.
    #
    # Depending on the system characteristics, some combinations of
    # sizes, formats and stream usages might or might not be possible.
    #
    # A Camera produces a CameraConfigration based on a set of intended
    # roles for each Stream the application requires.

    config = camera.generate_configuration([libcam.StreamRole.Viewfinder])

    # The CameraConfiguration contains a StreamConfiguration instance
    # for each StreamRole requested by the application, provided
    # the Camera can support all of them.
    #
    # Each StreamConfiguration has default size and format, assigned
    # by the Camera depending on the Role the application has requested.

    stream_config = config.at(0)
    print(f'Default viewfinder configuration is: {stream_config}')

    # Each StreamConfiguration parameter which is part of a
    # CameraConfiguration can be independently modified by the
    # application.
    #
    # In order to validate the modified parameter, the CameraConfiguration
    # should be validated -before- the CameraConfiguration gets applied
    # to the Camera.
    #
    # The CameraConfiguration validation process adjusts each
    # StreamConfiguration to a valid value.

    # Validating a CameraConfiguration -before- applying it will adjust it
    # to a valid configuration which is as close as possible to the one
    # requested.

    config.validate()
    print(f'Validated viewfinder configuration is: {stream_config}')

    # Once we have a validated configuration, we can apply it to the
    # Camera.

    camera.configure(config)

    # --------------------------------------------------------------------
    # Buffer Allocation
    #
    # Now that a camera has been configured, it knows all about its
    # Streams sizes and formats. The captured images need to be stored in
    # framebuffers which can either be provided by the application to the
    # library, or allocated in the Camera and exposed to the application
    # by libcamera.
    #
    # An application may decide to allocate framebuffers from elsewhere,
    # for example in memory allocated by the display driver that will
    # render the captured frames. The application will provide them to
    # libcamera by constructing FrameBuffer instances to capture images
    # directly into.
    #
    # Alternatively libcamera can help the application by exporting
    # buffers allocated in the Camera using a FrameBufferAllocator
    # instance and referencing a configured Camera to determine the
    # appropriate buffer size and types to create.

    allocator = libcam.FrameBufferAllocator(camera)

    for cfg in config:
        ret = allocator.allocate(cfg.stream)
        if ret < 0:
            print('Can\'t allocate buffers')
            return -1

        allocated = len(allocator.buffers(cfg.stream))
        print(f'Allocated {allocated} buffers for stream')

    # --------------------------------------------------------------------
    # Frame Capture
    #
    # libcamera frames capture model is based on the 'Request' concept.
    # For each frame a Request has to be queued to the Camera.
    #
    # A Request refers to (at least one) Stream for which a Buffer that
    # will be filled with image data shall be added to the Request.
    #
    # A Request is associated with a list of Controls, which are tunable
    # parameters (similar to v4l2_controls) that have to be applied to
    # the image.
    #
    # Once a request completes, all its buffers will contain image data
    # that applications can access and for each of them a list of metadata
    # properties that reports the capture parameters applied to the image.

    stream = stream_config.stream
    buffers = allocator.buffers(stream)
    requests = []
    for i in range(len(buffers)):
        request = camera.create_request()
        if not request:
            print('Can\'t create request')
            return -1

        buffer = buffers[i]
        ret = request.add_buffer(stream, buffer)
        if ret < 0:
            print('Can\'t set buffer for request')
            return -1

        # Controls can be added to a request on a per frame basis.
        request.set_control(libcam.controls.Brightness, 0.5)

        requests.append(request)

    # --------------------------------------------------------------------
    # Start Capture
    #
    # In order to capture frames the Camera has to be started and
    # Request queued to it. Enough Request to fill the Camera pipeline
    # depth have to be queued before the Camera start delivering frames.
    #
    # When a Request has been completed, it will be added to a list in the
    # CameraManager and an event will be raised using eventfd.
    #
    # The list of completed Requests can be retrieved with
    # CameraManager.get_ready_requests(), which will also clear the list in the
    # CameraManager.
    #
    # The eventfd can be retrieved from CameraManager.event_fd, and the fd can
    # be waited upon using e.g. Python's selectors.

    camera.start()
    for request in requests:
        camera.queue_request(request)

    sel = selectors.DefaultSelector()
    sel.register(cm.event_fd, selectors.EVENT_READ, lambda fd: handle_camera_event(cm))

    start_time = time.time()

    while time.time() - start_time < TIMEOUT_SEC:
        events = sel.select()
        for key, mask in events:
            key.data(key.fileobj)

    # --------------------------------------------------------------------
    # Clean Up
    #
    # Stop the Camera, release resources and stop the CameraManager.
    # libcamera has now released all resources it owned.

    camera.stop()
    camera.release()

    return 0


if __name__ == '__main__':
    sys.exit(main())
