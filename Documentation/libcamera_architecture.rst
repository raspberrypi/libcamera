.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: documentation-contents.rst

libcamera Architecture
======================

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
  :doc:`Pipeline Handler Writers Guide <guides/pipeline-handler>`.

.. _PipelineHandler API: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html

Image Processing Algorithms
  Together with the hardware image processing and hardware statistics
  collection, the Image Processing Algorithms (IPA) implement 3A (Auto-Exposure,
  Auto-White Balance and Auto-Focus) and other algorithms. They run on the CPU
  and control hardware image processing based on the parameters supplied by
  upper layers, closing the control loop of the ISP.

  IPAs are loaded as external plugins named IPA Modules. IPA Modules can be part
  of the libcamera code base or provided externally by camera vendors as
  open-source or closed-source components.

  Open source IPA Modules built with libcamera are run in the same process space
  as libcamera. External IPA Modules are run in a separate sandboxed process. In
  either case, they can only interact with libcamera through the API provided by
  the Pipeline Handler. They have a restricted view of the system, with no direct
  access to kernel camera devices, no access to networking APIs, and limited
  access to file systems. All their accesses to image and metadata are mediated
  by dmabuf instances explicitly passed by the Pipeline Handler to the IPA
  Module.

  IPA Modules are only required for platforms and devices with an ISP controlled
  by the host CPU. Camera sensors which have an integrated ISP are not
  controlled through the IPA Module.

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

Platform Support
----------------

The library currently supports the following hardware platforms specifically
with dedicated pipeline handlers:

   - Arm Mali-C55
   - Intel IPU3 (ipu3)
   - NXP i.MX8MP (imx8-isi and rkisp1)
   - RaspberryPi 3, 4 and zero (rpi/vc4)
   - Rockchip RK3399 (rkisp1)

Furthermore, generic platform support is provided for the following:

   - USB video device class cameras (uvcvideo)
   - iMX7, IPU6, Allwinner Sun6i (simple)
   - Virtual media controller driver for test use cases (vimc)
