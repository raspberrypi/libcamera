.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: documentation-contents.rst

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
