.. SPDX-License-Identifier: CC-BY-SA-4.0

.. contents::
   :local:

*************
Documentation
*************

.. toctree::
   :hidden:

   API <api-html/index>

API
===

The libcamera API is extensively documented using Doxygen. The :ref:`API
nightly build <api>` contains the most up-to-date API documentation, built from
the latest master branch.

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

The camera devices shall implement auto exposure, auto gain and auto white
balance. Camera devices that include a focus lens shall implement auto
focus. Additional image enhancement algorithms, such as noise reduction or
video stabilization, may be implemented.

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


Camera Stack
============

::

    a c /    +-------------+  +-------------+  +-------------+  +-------------+
    p a |    |   Native    |  |  Framework  |  |   Native    |  |   Android   |
    p t |    |    V4L2     |  | Application |  |  libcamera  |  |   Camera    |
    l i |    | Application |  | (gstreamer) |  | Application |  |  Framework  |
    i o \    +-------------+  +-------------+  +-------------+  +-------------+
      n             ^                ^                ^                ^
                    |                |                |                |
    l a             |                |                |                |
    i d             v                v                |                v
    b a /    +-------------+  +-------------+         |         +-------------+
    c p |    |    V4L2     |  |   Camera    |         |         |   Android   |
    a t |    |   Compat.   |  |  Framework  |         |         |   Camera    |
    m a |    |             |  | (gstreamer) |         |         |     HAL     |
    e t \    +-------------+  +-------------+         |         +-------------+
    r i             ^                ^                |                ^
    a o             |                |                |                |
      n             |                |                |                |
        /           |         ,................................................
        |           |         !      :            Language             :      !
    l f |           |         !      :            Bindings             :      !
    i r |           |         !      :           (optional)            :      !
    b a |           |         \...............................................'
    c m |           |                |                |                |
    a e |           |                |                |                |
    m w |           v                v                v                v
    e o |    +----------------------------------------------------------------+
    r r |    |                                                                |
    a k |    |                           libcamera                            |
        |    |                                                                |
        \    +----------------------------------------------------------------+
                            ^                  ^                  ^
    Userspace               |                  |                  |
   ------------------------ | ---------------- | ---------------- | ---------------
    Kernel                  |                  |                  |
                            v                  v                  v
                      +-----------+      +-----------+      +-----------+
                      |   Media   | <--> |   Video   | <--> |   V4L2    |
                      |  Device   |      |  Device   |      |  Subdev   |
                      +-----------+      +-----------+      +-----------+

The camera stack comprises four software layers. From bottom to top:

* The kernel drivers control the camera hardware and expose a
  low-level interface to userspace through the Linux kernel V4L2
  family of APIs (Media Controller API, V4L2 Video Device API and
  V4L2 Subdev API).

* The libcamera framework is the core part of the stack. It
  handles all control of the camera devices in its core component,
  libcamera, and exposes a native C++ API to upper layers. Optional
  language bindings allow interfacing to libcamera from other
  programming languages.

  Those components live in the same source code repository and
  all together constitute the libcamera framework.

* The libcamera adaptation is an umbrella term designating the
  components that interface to libcamera in other frameworks.
  Notable examples are a V4L2 compatibility layer, a gstreamer
  libcamera element, and an Android camera HAL implementation based
  on libcamera.

  Those components can live in the libcamera project source code
  in separate repositories, or move to their respective project's
  repository (for instance the gstreamer libcamera element).

* The applications and upper level frameworks are based on the
  libcamera framework or libcamera adaptation, and are outside of
  the scope of the libcamera project.


libcamera Architecture
======================

::

   ---------------------------< libcamera Public API >---------------------------
                    ^                                      ^
                    |                                      |
                    v                                      v
             +-------------+  +-------------------------------------------------+
             |   Camera    |  |  Camera Device                                  |
             |   Devices   |  | +---------------------------------------------+ |
             |   Manager   |  | | Device-Agnostic                             | |
             +-------------+  | |                                             | |
                    ^         | |                    +------------------------+ |
                    |         | |                    |   ~~~~~~~~~~~~~~~~~~~~~  |
                    |         | |                    |  {  +---------------+  } |
                    |         | |                    |  }  | ////Image//// |  { |
                    |         | |                    | <-> | /Processing// |  } |
                    |         | |                    |  }  | /Algorithms// |  { |
                    |         | |                    |  {  +---------------+  } |
                    |         | |                    |   ~~~~~~~~~~~~~~~~~~~~~  |
                    |         | |                    | ======================== |
                    |         | |                    |     +---------------+    |
                    |         | |                    |     | //Pipeline/// |    |
                    |         | |                    | <-> | ///Handler/// |    |
                    |         | |                    |     | ///////////// |    |
                    |         | +--------------------+     +---------------+    |
                    |         |                                 Device-Specific |
                    |         +-------------------------------------------------+
                    |                     ^                        ^
                    |                     |                        |
                    v                     v                        v
           +--------------------------------------------------------------------+
           | Helpers and Support Classes                                        |
           | +-------------+  +-------------+  +-------------+  +-------------+ |
           | |  MC & V4L2  |  |   Buffers   |  | Sandboxing  |  |   Plugins   | |
           | |   Support   |  |  Allocator  |  |     IPC     |  |   Manager   | |
           | +-------------+  +-------------+  +-------------+  +-------------+ |
           | +-------------+  +-------------+                                   |
           | |  Pipeline   |  |     ...     |                                   |
           | |   Runner    |  |             |                                   |
           | +-------------+  +-------------+                                   |
           +--------------------------------------------------------------------+

             /// Device-Specific Components
             ~~~ Sandboxing

While offering a unified API towards upper layers, and presenting
itself as a single library, libcamera isn't monolithic. It exposes
multiple components through its public API, is built around a set of
separate helpers internally, uses device-specific components and can
load dynamic plugins.

Camera Devices Manager
  The Camera Devices Manager provides a view of available cameras
  in the system. It performs cold enumeration and runtime camera
  management, and supports a hotplug notification mechanism in its
  public API.

  To avoid the cost associated with cold enumeration of all devices
  at application start, and to arbitrate concurrent access to camera
  devices, the Camera Devices Manager could later be split to a
  separate service, possibly with integration in platform-specific
  device management.

Camera Device
  The Camera Device represents a camera device to upper layers. It
  exposes full control of the device through the public API, and is
  thus the highest level object exposed by libcamera.

  Camera Device instances are created by the Camera Devices
  Manager. An optional function to create new instances could be exposed
  through the public API to speed up initialization when the upper
  layer knows how to directly address camera devices present in the
  system.

Pipeline Handler
  The Pipeline Handler manages complex pipelines exposed by the kernel drivers
  through the Media Controller and V4L2 APIs. It abstracts pipeline handling to
  hide device-specific details to the rest of the library, and implements both
  pipeline configuration based on stream configuration, and pipeline runtime
  execution and scheduling when needed by the device.

  This component is device-specific and is part of the libcamera code base. As
  such it is covered by the same free software license as the rest of libcamera
  and needs to be contributed upstream by device vendors. The Pipeline Handler
  lives in the same process as the rest of the library, and has access to all
  helpers and kernel camera-related devices.

Image Processing Algorithms
  Together with the hardware image processing and hardware statistics
  collection, the Image Processing Algorithms implement 3A (Auto-Exposure,
  Auto-White Balance and Auto-Focus) and other algorithms. They run on the CPU
  and interact with the kernel camera devices to control hardware image
  processing based on the parameters supplied by upper layers, closing the
  control loop of the ISP.

  This component is device-specific and is loaded as an external plugin. It can
  be part of the libcamera code base, in which case it is covered by the same
  license, or provided externally as an open-source or closed-source component.

  The component is sandboxed and can only interact with libcamera through
  internal APIs specifically marked as such. In particular it will have no
  direct access to kernel camera devices, and all its accesses to image and
  metadata will be mediated by dmabuf instances explicitly passed to the
  component. The component must be prepared to run in a process separate from
  the main libcamera process, and to have a very restricted view of the system,
  including no access to networking APIs and limited access to file systems.

  The sandboxing mechanism isn't defined by libcamera. One example
  implementation will be provided as part of the project, and platforms vendors
  will be able to provide their own sandboxing mechanism as a plugin.

  libcamera should provide a basic implementation of Image Processing
  Algorithms, to serve as a reference for the internal API. Device vendors are
  expected to provide a full-fledged implementation compatible with their
  Pipeline Handler. One goal of the libcamera project is to create an
  environment in which the community will be able to compete with the
  closed-source vendor binaries and develop a high quality open source
  implementation.

Helpers and Support Classes
  While Pipeline Handlers are device-specific, implementations are expected to
  share code due to usage of identical APIs towards the kernel camera drivers
  and the Image Processing Algorithms. This includes without limitation handling
  of the MC and V4L2 APIs, buffer management through dmabuf, and pipeline
  discovery, configuration and scheduling. Such code will be factored out to
  helpers when applicable.

  Other parts of libcamera will also benefit from factoring code out to
  self-contained support classes, even if such code is present only once in the
  code base, in order to keep the source code clean and easy to read. This
  should be the case for instance for plugin management.


V4L2 Compatibility Layer
------------------------

V4L2 compatibility is achieved through a shared library that traps all
accesses to camera devices and routes them to libcamera to emulate high-level
V4L2 camera devices. It is injected in a process address space through
`LD_PRELOAD` and is completely transparent for applications.

The compatibility layer exposes camera device features on a best-effort basis,
and aims for the level of features traditionally available from a UVC camera
designed for video conferencing.


Android Camera HAL
------------------

Camera support for Android is achieved through a generic Android
camera HAL implementation on top of libcamera. The HAL will implement internally
features required by Android and missing from libcamera, such as JPEG encoding
support.

The Android camera HAL implementation will initially target the
LIMITED hardware level, with support for the FULL level then being gradually
implemented.
