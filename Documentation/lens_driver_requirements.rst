.. SPDX-License-Identifier: CC-BY-SA-4.0

.. _lens-driver-requirements:

Lens Driver Requirements
========================

libcamera handles lens devices in the CameraLens class and defines
a consistent interface through its API towards other library components.

The CameraLens class uses the V4L2 subdev kernel API to interface with the
camera lens through a sub-device exposed to userspace by the lens driver.

In order for libcamera to be fully operational and provide all the required
information to interface with the camera lens to applications and pipeline
handlers, a set of mandatory features the driver has to support has been defined.

Mandatory Requirements
----------------------

The lens driver is assumed to be fully compliant with the V4L2 specification.

The lens driver shall support the following V4L2 controls:

* `V4L2_CID_FOCUS_ABSOLUTE`_

.. _V4L2_CID_FOCUS_ABSOLUTE: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/ext-ctrls-camera.html
