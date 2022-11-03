.. SPDX-License-Identifier: CC-BY-SA-4.0

.. _sensor-driver-requirements:

Sensor Driver Requirements
==========================

libcamera handles imaging devices in the CameraSensor class and defines
a consistent interface through its API towards other library components.

The CameraSensor class uses the V4L2 subdev kernel API to interface with the
camera sensor through one or multiple sub-devices exposed in userspace by
the sensor driver.

In order for libcamera to be fully operational and provide all the required
information to interface with the camera sensor to applications and pipeline
handlers, a set of mandatory and optional features the driver has to support
has been defined.

Mandatory Requirements
----------------------

The sensor driver is assumed to be fully compliant with the V4L2 specification.

For RAW sensors, the sensor driver shall support the following V4L2 controls:

* `V4L2_CID_ANALOGUE_GAIN`_
* `V4L2_CID_EXPOSURE`_
* `V4L2_CID_HBLANK`_
* `V4L2_CID_PIXEL_RATE`_
* `V4L2_CID_VBLANK`_

.. _V4L2_CID_ANALOGUE_GAIN: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-image-source.html
.. _V4L2_CID_EXPOSURE: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/control.html
.. _V4L2_CID_HBLANK: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-image-source.html
.. _V4L2_CID_PIXEL_RATE: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-image-process.html
.. _V4L2_CID_VBLANK: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-image-source.html

The ``ANALOGUE_GAIN`` control units are sensor-specific. libcamera requires
a sensor-specific CameraSensorHelper implementation to translate between the
sensor specific ``gain code`` and the analogue ``gain value`` expressed as an
absolute number as defined by ``controls::AnalogueGain``.

While V4L2 doesn't specify a unit for the ``EXPOSURE`` control, libcamera
requires it to be expressed as a number of image lines. Camera sensor drivers
that do not comply with this requirement will need to be adapted or will produce
incorrect results.

The ``HBLANK``, ``PIXEL_RATE`` and ``VBLANK`` controls are used to compute the
sensor output timings.

Optional Requirements
---------------------

The sensor driver should support the following V4L2 controls:

* `V4L2_CID_CAMERA_ORIENTATION`_
* `V4L2_CID_CAMERA_SENSOR_ROTATION`_

.. _V4L2_CID_CAMERA_ORIENTATION: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-camera.html
.. _V4L2_CID_CAMERA_SENSOR_ROTATION: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-camera.html

The controls are used to register the camera location and rotation.

The sensor driver should implement support for the V4L2 Selection API,
specifically it should implement support for the
`VIDIOC_SUBDEV_G_SELECTION`_ ioctl with support for the following selection
targets:

.. _VIDIOC_SUBDEV_G_SELECTION: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/vidioc-subdev-g-selection.html#c.V4L.VIDIOC_SUBDEV_G_SELECTION

* `V4L2_SEL_TGT_CROP_BOUNDS`_ to report the readable pixel array area size
* `V4L2_SEL_TGT_CROP_DEFAULT`_ to report the active pixel array area size
* `V4L2_SEL_TGT_CROP`_ to report the analogue selection rectangle

Support for the selection API is scheduled to become a mandatory feature in
the near future.

.. _V4L2_SEL_TGT_CROP_BOUNDS: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2-selection-targets.html
.. _V4L2_SEL_TGT_CROP_DEFAULT: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2-selection-targets.html
.. _V4L2_SEL_TGT_CROP: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2-selection-targets.html
