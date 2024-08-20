.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: ../documentation-contents.rst

Developers guide to libcamera
=============================

The Linux kernel handles multimedia devices through the 'Linux media' subsystem
and provides a set of APIs (application programming interfaces) known
collectively as V4L2 (`Video for Linux 2`_) and the `Media Controller`_ API
which provide an interface to interact and control media devices.

Included in this subsystem are drivers for camera sensors, CSI2 (Camera
Serial Interface) receivers, and ISPs (Image Signal Processors)

The usage of these drivers to provide a functioning camera stack is a
responsibility that lies in userspace which is commonly implemented separately
by vendors without a common architecture or API for application developers.

libcamera provides a complete camera stack for Linux based systems to abstract
functionality desired by camera application developers and process the
configuration of hardware and image control algorithms required to obtain
desirable results from the camera.

.. _Video for Linux 2: https://www.linuxtv.org/downloads/v4l-dvb-apis-new/userspace-api/v4l/v4l2.html
.. _Media Controller: https://www.linuxtv.org/downloads/v4l-dvb-apis-new/userspace-api/mediactl/media-controller.html


In this developers guide the `Licensing`_ requirements of the project are
detailed.

This introduction is followed by a walkthrough tutorial to newcomers wishing to
support a new platform with the `Pipeline Handler Writers Guide`_ and for those
looking to make use of the libcamera native API an `Application Writers Guide`_
provides a tutorial of the key APIs exposed by libcamera.

.. _Pipeline Handler Writers Guide: pipeline-handler.html
.. _Application Writers Guide: application-developer.html

.. TODO: Correctly link to the other articles of the guide

Licensing
---------

The libcamera core, is covered by the `LGPL-2.1-or-later`_ license. Pipeline
Handlers are a part of the libcamera code base and need to be contributed
upstream by device vendors. IPA modules included in libcamera are covered by a
free software license, however third-parties may develop IPA modules outside of
libcamera and distribute them under a closed-source license, provided they do
not include source code from the libcamera project.

The libcamera project itself contains multiple libraries, applications and
utilities. Licenses are expressed through SPDX tags in text-based files that
support comments, and through the .reuse/dep5 file otherwise. A copy of all
licenses are stored in the LICENSES directory, and a full summary of the
licensing used throughout the project can be found in the COPYING.rst document.

Applications which link dynamically against libcamera and use only the public
API are an independent work of the authors and have no license restrictions
imposed upon them from libcamera.

.. _LGPL-2.1-or-later: https://spdx.org/licenses/LGPL-2.1-or-later.html
