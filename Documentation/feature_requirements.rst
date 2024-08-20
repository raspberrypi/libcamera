.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: documentation-contents.rst

Feature Requirements
====================

Device enumeration
------------------

The library shall support enumerating all camera devices available in the
system, including both fixed cameras and hotpluggable cameras. It shall
support cameras plugged and unplugged after the initialization of the
library, and shall offer a mechanism to notify applications of camera plug
and unplug.

The following types of cameras shall be supported:

* Internal cameras designed for point-and-shoot still image and video
  capture usage, either controlled directly by the CPU, or exposed through
  an internal USB bus as a UVC device.

* External UVC cameras designed for video conferencing usage.

Other types of camera, including analog cameras, depth cameras, thermal
cameras, external digital picture or movie cameras, are out of scope for
this project.

A hardware device that includes independent camera sensors, such as front
and back sensors in a phone, shall be considered as multiple camera devices
for the purpose of this library.

Independent Camera Devices
--------------------------

When multiple cameras are present in the system and are able to operate
independently from each other, the library shall expose them as multiple
camera devices and support parallel operation without any additional usage
restriction apart from the limitations inherent to the hardware (such as
memory bandwidth, CPU usage or number of CSI-2 receivers for instance).

Independent processes shall be able to use independent cameras devices
without interfering with each other. A single camera device shall be
usable by a single process at a time.

Multiple streams support
------------------------

The library shall support multiple video streams running in parallel
for each camera device, within the limits imposed by the system.

Per frame controls
------------------

The library shall support controlling capture parameters for each stream
on a per-frame basis, on a best effort basis based on the capabilities of the
hardware and underlying software stack (including kernel drivers and
firmware). It shall apply capture parameters to the frame they target, and
report the value of the parameters that have effectively been used for each
captured frame.

When a camera device supports multiple streams, the library shall allow both
control of each stream independently, and control of multiple streams
together. Streams that are controlled together shall be synchronized. No
synchronization is required for streams controlled independently.

Capability Enumeration
----------------------

The library shall expose capabilities of each camera device in a way that
allows applications to discover those capabilities dynamically. Applications
shall be allowed to cache capabilities for as long as they are using the
library. If capabilities can change at runtime, the library shall offer a
mechanism to notify applications of such changes. Applications shall not
cache capabilities in long term storage between runs.

Capabilities shall be discovered dynamically at runtime from the device when
possible, and may come, in part or in full, from platform configuration
data.

Device Profiles
---------------

The library may define different camera device profiles, each with a minimum
set of required capabilities. Applications may use those profiles to quickly
determine the level of features exposed by a device without parsing the full
list of capabilities. Camera devices may implement additional capabilities
on top of the minimum required set for the profile they expose.

3A and Image Enhancement Algorithms
-----------------------------------

The library shall provide a basic implementation of Image Processing Algorithms
to serve as a reference for the internal API. This shall including auto exposure
and gain and auto white balance. Camera devices that include a focus lens shall
implement auto focus. Additional image enhancement algorithms, such as noise
reduction or video stabilization, may be implemented. Device vendors are
expected to provide a fully-fledged implementation compatible with their
Pipeline Handler. One goal of the libcamera project is to create an environment
in which the community will be able to compete with the closed-source vendor
biaries and develop a high quality open source implementation.

All algorithms may be implemented in hardware or firmware outside of the
library, or in software in the library. They shall all be controllable by
applications.

The library shall be architectured to isolate the 3A and image enhancement
algorithms in a component with a documented API, respectively called the 3A
component and the 3A API. The 3A API shall be stable, and shall allow both
open-source and closed-source implementations of the 3A component.

The library may include statically-linked open-source 3A components, and
shall support dynamically-linked open-source and closed-source 3A
components.

Closed-source 3A Component Sandboxing
-------------------------------------

For security purposes, it may be desired to run closed-source 3A components
in a separate process. The 3A API would in such a case be transported over
IPC. The 3A API shall make it possible to use any IPC mechanism that
supports passing file descriptors.

The library may implement an IPC mechanism, and shall support third-party
platform-specific IPC mechanisms through the implementation of a
platform-specific 3A API wrapper. No modification to the library shall be
needed to use such third-party IPC mechanisms.

The 3A component shall not directly access any device node on the system.
Such accesses shall instead be performed through the 3A API. The library
shall validate all accesses and restrict them to what is absolutely required
by 3A components.

V4L2 Compatibility Layer
------------------------

The project shall support traditional V4L2 application through an additional
libcamera wrapper library. The wrapper library shall trap all accesses to
camera devices through `LD_PRELOAD`, and route them through libcamera to
emulate a high-level V4L2 camera device. It shall expose camera device
features on a best-effort basis, and aim for the level of features
traditionally available from a UVC camera designed for video conferencing.

Android Camera HAL v3 Compatibility
-----------------------------------

The library API shall expose all the features required to implement an
Android Camera HAL v3 on top of libcamera. Some features of the HAL may be
omitted as long as they can be implemented separately in the HAL, such as
JPEG encoding, or YUV reprocessing.
