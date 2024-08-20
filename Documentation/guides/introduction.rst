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


In this developers guide, we will explore the internal `Architecture`_ of
the libcamera library with its components. The current `Platform Support`_ is
detailed, as well as an overview of the `Licensing`_ requirements of the
project.

This introduction is followed by a walkthrough tutorial to newcomers wishing to
support a new platform with the `Pipeline Handler Writers Guide`_ and for those
looking to make use of the libcamera native API an `Application Writers Guide`_
provides a tutorial of the key APIs exposed by libcamera.

.. _Pipeline Handler Writers Guide: pipeline-handler.html
.. _Application Writers Guide: application-developer.html

.. TODO: Correctly link to the other articles of the guide

Architecture
------------

While offering a unified API towards upper layers, and presenting itself as a
single library, libcamera isn't monolithic. It exposes multiple components
through its public API and is built around a set of separate helpers internally.
Hardware abstractions are handled through the use of device-specific components
where required and dynamically loadable plugins are used to separate image
processing algorithms from the core libcamera codebase.

::

   --------------------------< libcamera Public API >---------------------------
                 ^                                          ^
                 |                                          |
                 v                                          v
          +-------------+  +---------------------------------------------------+
          |   Camera    |  |  Camera Device                                    |
          |   Manager   |  | +-----------------------------------------------+ |
          +-------------+  | | Device-Agnostic                               | |
                 ^         | |                                               | |
                 |         | |                    +--------------------------+ |
                 |         | |                    |   ~~~~~~~~~~~~~~~~~~~~~~~  |
                 |         | |                    |  {  +-----------------+  } |
                 |         | |                    |  }  | //// Image //// |  { |
                 |         | |                    | <-> | / Processing // |  } |
                 |         | |                    |  }  | / Algorithms // |  { |
                 |         | |                    |  {  +-----------------+  } |
                 |         | |                    |   ~~~~~~~~~~~~~~~~~~~~~~~  |
                 |         | |                    | ========================== |
                 |         | |                    |     +-----------------+    |
                 |         | |                    |     | // Pipeline /// |    |
                 |         | |                    | <-> | /// Handler /// |    |
                 |         | |                    |     | /////////////// |    |
                 |         | +--------------------+     +-----------------+    |
                 |         |                                   Device-Specific |
                 |         +---------------------------------------------------+
                 |                          ^                         ^
                 |                          |                         |
                 v                          v                         v
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


Camera Manager
  The Camera Manager enumerates cameras and instantiates Pipeline Handlers to
  manage each Camera that libcamera supports. The Camera Manager supports
  hotplug detection and notification events when supported by the underlying
  kernel devices.

  There is only ever one instance of the Camera Manager running per application.
  Each application's instance of the Camera Manager ensures that only a single
  application can take control of a camera device at once.

  Read the `Camera Manager API`_ documentation for more details.

.. _Camera Manager API: https://libcamera.org/api-html/classlibcamera_1_1CameraManager.html

Camera Device
  The Camera class represents a single item of camera hardware that is capable
  of producing one or more image streams, and provides the API to interact with
  the underlying device.

  If a system has multiple instances of the same hardware attached, each has its
  own instance of the camera class.

  The API exposes full control of the device to upper layers of libcamera through
  the public API, making it the highest level object libcamera exposes, and the
  object that all other API operations interact with from configuration to
  capture.

  Read the `Camera API`_ documentation for more details.

.. _Camera API: https://libcamera.org/api-html/classlibcamera_1_1Camera.html

Pipeline Handler
  The Pipeline Handler manages the complex pipelines exposed by the kernel
  drivers through the Media Controller and V4L2 APIs. It abstracts pipeline
  handling to hide device-specific details from the rest of the library, and
  implements both pipeline configuration based on stream configuration, and
  pipeline runtime execution and scheduling when needed by the device.

  The Pipeline Handler lives in the same process as the rest of the library, and
  has access to all helpers and kernel camera-related devices.

  Hardware abstraction is handled by device specific Pipeline Handlers which are
  derived from the Pipeline Handler base class allowing commonality to be shared
  among the implementations.

  Derived pipeline handlers create Camera device instances based on the devices
  they detect and support on the running system, and are responsible for
  managing the interactions with a camera device.

  More details can be found in the `PipelineHandler API`_ documentation, and the
  `Pipeline Handler Writers Guide`_.

.. _PipelineHandler API: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html

Image Processing Algorithms
  An image processing algorithm (IPA) component is a loadable plugin that
  implements 3A (Auto-Exposure, Auto-White Balance, and Auto-Focus) and other
  algorithms.

  The algorithms run on the CPU and interact with the camera devices through the
  Pipeline Handler to control hardware image processing based on the parameters
  supplied by upper layers, maintaining state and closing the control loop
  of the ISP.

  The component is sandboxed and can only interact with libcamera through the
  API provided by the Pipeline Handler and an IPA has no direct access to kernel
  camera devices.

  Open source IPA modules built with libcamera can be run in the same process
  space as libcamera, however external IPA modules are run in a separate process
  from the main libcamera process. IPA modules have a restricted view of the
  system, including no access to networking APIs and limited access to file
  systems.

  IPA modules are only required for platforms and devices with an ISP controlled
  by the host CPU. Camera sensors which have an integrated ISP are not
  controlled through the IPA module.

Platform Support
----------------

The library currently supports the following hardware platforms specifically
with dedicated pipeline handlers:

   -  Intel IPU3 (ipu3)
   -  Rockchip RK3399 (rkisp1)
   -  RaspberryPi 3 and 4 (rpi/vc4)

Furthermore, generic platform support is provided for the following:

   -  USB video device class cameras (uvcvideo)
   -  iMX7, Allwinner Sun6i (simple)
   -  Virtual media controller driver for test use cases (vimc)

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
