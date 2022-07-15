.. SPDX-License-Identifier: CC-BY-SA-4.0

Pipeline Handler Writers Guide
==============================

Pipeline handlers are the abstraction layer for device-specific hardware
configuration. They access and control hardware through the V4L2 and Media
Controller kernel interfaces, and implement an internal API to control the ISP
and capture components of a pipeline directly.

Prerequisite knowledge: system architecture
-------------------------------------------

A pipeline handler configures and manages the image acquisition and
transformation pipeline realized by specialized system peripherals combined with
an image source connected to the system through a data and control bus. The
presence, number and characteristics of them vary depending on the system design
and the product integration of the target platform.

System components can be classified in three macro-categories:

.. TODO: Insert references to the open CSI-2 (and other) specification.

- Input ports: Interfaces to external devices, usually image sensors,
  which transfer data from the physical bus to locations accessible by other
  system peripherals. An input port needs to be configured according to the
  input image format and size and could optionally apply basic transformations
  on the received images, most typically cropping/scaling and some formats
  conversion. The industry standard for the system typically targeted by
  libcamera is to have receivers compliant with the MIPI CSI-2 specifications,
  implemented on a compatible physical layer such as MIPI D-PHY or MIPI C-PHY.
  Other design are possible but less common, such as LVDS or the legacy BT.601
  and BT.656 parallel protocols.

- Image Signal Processor (ISP): A specialized media processor which applies
  digital transformations on image streams. ISPs can be integrated as part of
  the SoC as a memory interfaced system peripheral or packaged as stand-alone
  chips connected to the application processor through a bus. Most hardware used
  by libcamera makes use of in-system ISP designs but pipelines can equally
  support external ISP chips or be instrumented to use other system resources
  such as a GPU or an FPGA IP block. ISPs expose a software programming
  interface that allows the configuration of multiple processing blocks which
  form an "Image Transformation Pipeline". An ISP usually produces 'processed'
  image streams along with the metadata describing the processing steps which
  have been applied to generate the output frames.

- Camera Sensor: Digital components that integrate an image sensor with control
  electronics and usually a lens. It interfaces to the SoC image receiver ports
  and is programmed to produce images in a format and size suitable for the
  current system configuration. Complex camera modules can integrate on-board
  ISP or DSP chips and process images before delivering them to the system. Most
  systems with a dedicated ISP processor will usually integrate camera sensors
  which produce images in Raw Bayer format and defer processing to it.

It is the responsibility of the pipeline handler to interface with these (and
possibly other) components of the system and implement the following
functionalities:

- Detect and register camera devices available in the system with an associated
  set of image streams.

- Configure the image acquisition and processing pipeline by assigning the
  system resources (memory, shared components, etc.) to satisfy the
  configuration requested by the application.

- Start and stop the image acquisition and processing sessions.

- Apply configuration settings requested by applications and computed by image
  processing algorithms integrated in libcamera to the hardware devices.

- Notify applications of the availability of new images and deliver them to the
  correct locations.

Prerequisite knowledge: libcamera architecture
----------------------------------------------

A pipeline handler makes use of the following libcamera classes to realize the
functionalities described above. Below is a brief overview of each of those:

.. TODO: (All) Convert to sphinx refs
.. TODO: (MediaDevice) Reference to the Media Device API (possibly with versioning requirements)
.. TODO: (IPAInterface) refer to the IPA guide

-  `MediaDevice <https://libcamera.org/api-html/classlibcamera_1_1MediaDevice.html>`_:
   Instances of this class are associated with a kernel media controller
   device and its connected objects.

-  `DeviceEnumerator <https://libcamera.org/api-html/classlibcamera_1_1DeviceEnumerator.html>`_:
   Enumerates all media devices attached to the system and the media entities
   registered with it, by creating instances of the ``MediaDevice`` class and
   storing them.

-  `DeviceMatch <https://libcamera.org/api-html/classlibcamera_1_1DeviceMatch.html>`_:
   Describes a media device search pattern using entity names, or other
   properties.

-  `V4L2VideoDevice <https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html>`_:
   Models an instance of a V4L2 video device constructed with the path to a V4L2
   video device node.

-  `V4L2SubDevice <https://libcamera.org/api-html/classlibcamera_1_1V4L2Subdevice.html>`_:
   Provides an API to the sub-devices that model the hardware components of a
   V4L2 device.

-  `CameraSensor <https://libcamera.org/api-html/classlibcamera_1_1CameraSensor.html>`_:
   Abstracts camera sensor handling by hiding the details of the V4L2 subdevice
   kernel API and caching sensor information.

-  `Camera::Private <https://libcamera.org/api-html/classlibcamera_1_1Camera_1_1Private.html>`_:
   Represents device-specific data a pipeline handler associates to each Camera
   instance.

-  `StreamConfiguration <https://libcamera.org/api-html/structlibcamera_1_1StreamConfiguration.html>`_:
   Models the current configuration of an image stream produced by the camera by
   reporting its format and sizes.

-  `CameraConfiguration <https://libcamera.org/api-html/classlibcamera_1_1CameraConfiguration.html>`_:
   Represents the current configuration of a camera, which includes a list of
   stream configurations for each active stream in a capture session. When
   validated, it is applied to the camera.

-  `IPAInterface <https://libcamera.org/api-html/classlibcamera_1_1IPAInterface.html>`_:
   The interface to the Image Processing Algorithm (IPA) module which performs
   the computation of the image processing pipeline tuning parameters.

-  `ControlList <https://libcamera.org/api-html/classlibcamera_1_1ControlList.html>`_:
   A list of control items, indexed by Control<> instances or by numerical index
   which contains values used by application and IPA to change parameters of
   image streams, used to return to applications and share with IPA the metadata
   associated with the captured images, and to advertise the immutable camera
   characteristics enumerated at system initialization time.

Creating a PipelineHandler
--------------------------

This guide walks through the steps to create a simple pipeline handler
called “Vivid” that supports the `V4L2 Virtual Video Test Driver`_ (vivid).

To use the vivid test driver, you first need to check that the vivid kernel
module is loaded, for example with the ``modprobe vivid`` command.

.. _V4L2 Virtual Video Test Driver: https://www.kernel.org/doc/html/latest/admin-guide/media/vivid.html

Create the skeleton file structure
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To add a new pipeline handler, create a directory to hold the pipeline code in
the *src/libcamera/pipeline/* directory that matches the name of the pipeline
(in this case *vivid*). Inside the new directory add a *meson.build* file that
integrates with the libcamera build system, and a *vivid.cpp* file that matches
the name of the pipeline.

In the *meson.build* file, add the *vivid.cpp* file as a build source for
libcamera by adding it to the global meson ``libcamera_sources`` variable:

.. code-block:: none

   # SPDX-License-Identifier: CC0-1.0

   libcamera_sources += files([
       'vivid.cpp',
   ])

Users of libcamera can selectively enable pipelines while building libcamera
using the ``pipelines`` option.

For example, to enable only the IPU3, UVC, and VIVID pipelines, specify them as
a comma separated list with ``-Dpipelines`` when generating a build directory:

.. code-block:: shell

    meson build -Dpipelines=ipu3,uvcvideo,vivid

Read the `Meson build configuration`_ documentation for more information on
configuring a build directory.

.. _Meson build configuration: https://mesonbuild.com/Configuring-a-build-directory.html

To add the new pipeline handler to this list of options, add its directory name
to the libcamera build options in the top level ``meson_options.txt``.

.. code-block:: none

   option('pipelines',
           type : 'array',
           choices : ['ipu3', 'raspberrypi', 'rkisp1', 'simple', 'uvcvideo', 'vimc', 'vivid'],
           description : 'Select which pipeline handlers to include')


In *vivid.cpp* add the pipeline handler to the ``libcamera`` namespace, defining
a `PipelineHandler`_ derived class named PipelineHandlerVivid, and add stub
implementations for the overridden class members.

.. _PipelineHandler: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html

.. code-block:: cpp

   namespace libcamera {

   class PipelineHandlerVivid : public PipelineHandler
   {
   public:
          PipelineHandlerVivid(CameraManager *manager);

          CameraConfiguration *generateConfiguration(Camera *camera,
          const StreamRoles &roles) override;
          int configure(Camera *camera, CameraConfiguration *config) override;

          int exportFrameBuffers(Camera *camera, Stream *stream,
          std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

          int start(Camera *camera, const ControlList *controls) override;
          void stop(Camera *camera) override;

          int queueRequestDevice(Camera *camera, Request *request) override;

          bool match(DeviceEnumerator *enumerator) override;
   };

   PipelineHandlerVivid::PipelineHandlerVivid(CameraManager *manager)
          : PipelineHandler(manager)
   {
   }

   CameraConfiguration *PipelineHandlerVivid::generateConfiguration(Camera *camera,
                                                                    const StreamRoles &roles)
   {
          return nullptr;
   }

   int PipelineHandlerVivid::configure(Camera *camera, CameraConfiguration *config)
   {
          return -1;
   }

   int PipelineHandlerVivid::exportFrameBuffers(Camera *camera, Stream *stream,
                                                std::vector<std::unique_ptr<FrameBuffer>> *buffers)
   {
          return -1;
   }

   int PipelineHandlerVivid::start(Camera *camera, const ControlList *controls)
   {
          return -1;
   }

   void PipelineHandlerVivid::stop(Camera *camera)
   {
   }

   int PipelineHandlerVivid::queueRequestDevice(Camera *camera, Request *request)
   {
          return -1;
   }

   bool PipelineHandlerVivid::match(DeviceEnumerator *enumerator)
   {
          return false;
   }

   REGISTER_PIPELINE_HANDLER(PipelineHandlerVivid)

   } /* namespace libcamera */

Note that you must register the ``PipelineHandler`` subclass with the pipeline
handler factory using the `REGISTER_PIPELINE_HANDLER`_ macro which
registers it and creates a global symbol to reference the class and make it
available to try and match devices.

.. _REGISTER_PIPELINE_HANDLER: https://libcamera.org/api-html/pipeline__handler_8h.html

For debugging and testing a pipeline handler during development, you can define
a log message category for the pipeline handler. The ``LOG_DEFINE_CATEGORY``
macro and ``LIBCAMERA_LOG_LEVELS`` environment variable help you use the inbuilt
libcamera `logging infrastructure`_ that allow for the inspection of internal
operations in a user-configurable way.

.. _logging infrastructure: https://libcamera.org/api-html/log_8h.html

Add the following before the ``PipelineHandlerVivid`` class declaration:

.. code-block:: cpp

   LOG_DEFINE_CATEGORY(VIVID)

At this point you need the following includes for logging and pipeline handler
features:

.. code-block:: cpp

   #include <libcamera/base/log.h>

   #include "libcamera/internal/pipeline_handler.h"

Run the following commands:

.. code-block:: shell

   meson build
   ninja -C build

To build the libcamera code base, and confirm that the build system found the
new pipeline handler by running:

.. code-block:: shell

   LIBCAMERA_LOG_LEVELS=Camera:0 ./build/src/cam/cam -l

And you should see output like the below:

.. code-block:: shell

    DEBUG Camera camera_manager.cpp:148 Found registered pipeline handler 'PipelineHandlerVivid'

Matching devices
~~~~~~~~~~~~~~~~

Each pipeline handler registered in libcamera gets tested against the current
system configuration, by matching a ``DeviceMatch`` with the system
``DeviceEnumerator``. A successful match makes sure all the requested components
have been registered in the system and allows the pipeline handler to be
initialized.

The main entry point of a pipeline handler is the `match()`_ class member
function. When the ``CameraManager`` is started (using the `start()`_ function),
all the registered pipeline handlers are iterated and their ``match`` function
called with an enumerator of all devices it found on a system.

The match function should identify if there are suitable devices available in
the ``DeviceEnumerator`` which the pipeline supports, returning ``true`` if it
matches a device, and ``false`` if it does not. To do this, construct a
`DeviceMatch`_ class with the name of the ``MediaController`` device to match.
You can specify the search further by adding specific media entities to the
search using the ``.add()`` function on the DeviceMatch.

.. _match(): https://www.libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a7cd5b652a2414b543ec20ba9dabf61b6
.. _start(): https://libcamera.org/api-html/classlibcamera_1_1CameraManager.html#a49e322880a2a26013bb0076788b298c5
.. _DeviceMatch: https://libcamera.org/api-html/classlibcamera_1_1DeviceMatch.html

This example uses search patterns that match vivid, but when developing a new
pipeline handler, you should change this value to suit your device identifier.

Replace the contents of the ``PipelineHandlerVivid::match`` function with the
following:

.. code-block:: cpp

   DeviceMatch dm("vivid");
   dm.add("vivid-000-vid-cap");
   return false; // Prevent infinite loops for now

With the device matching criteria defined, attempt to acquire exclusive access
to the matching media controller device with the `acquireMediaDevice`_ function.
If the function attempts to acquire a device it has already matched, it returns
``false``.

.. _acquireMediaDevice: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a77e424fe704e7b26094164b9189e0f84

Add the following below ``dm.add("vivid-000-vid-cap");``:

.. code-block:: cpp

   MediaDevice *media = acquireMediaDevice(enumerator, dm);
   if (!media)
           return false;

The pipeline handler now needs an additional include. Add the following to the
existing include block for device enumeration functionality:

.. code-block:: cpp

   #include "libcamera/internal/device_enumerator.h"

At this stage, you should test that the pipeline handler can successfully match
the devices, but have not yet added any code to create a Camera which libcamera
reports to applications.

As a temporary validation step, add a debug print with

.. code-block:: cpp

   LOG(VIVID, Debug) << "Vivid Device Identified";

before the final closing return statement in the ``PipelineHandlerVivid::match``
function for when when the pipeline handler successfully matches the
``MediaDevice`` and ``MediaEntity`` names.

Test that the pipeline handler matches and finds a device by rebuilding, and
running

.. code-block:: shell

   ninja -C build
   LIBCAMERA_LOG_LEVELS=Pipeline,VIVID:0 ./build/src/cam/cam -l

And you should see output like the below:

.. code-block:: shell

    DEBUG VIVID vivid.cpp:74 Vivid Device Identified

Creating camera devices
~~~~~~~~~~~~~~~~~~~~~~~

If the pipeline handler successfully matches with the system it is running on,
it can proceed to initialization, by creating all the required instances of the
``V4L2VideoDevice``, ``V4L2Subdevice`` and ``CameraSensor`` hardware abstraction
classes. If the Pipeline handler supports an ISP, it can then also initialise
the IPA module before proceeding to the creation of the Camera devices.

An image ``Stream`` represents a sequence of images and data of known size and
format, stored in application-accessible memory locations. Typical examples of
streams are the ISP processed outputs and the raw images captured at the
receivers port output.

The Pipeline Handler is responsible for defining the set of Streams associated
with the Camera.

Each Camera has instance-specific data represented using the `Camera::Private`_
class, which can be extended for the specific needs of the pipeline handler.

.. _Camera::Private: https://libcamera.org/api-html/classlibcamera_1_1Camera_1_1Private.html


To support the Camera we will later register, we need to create a Camera::Private
class that we can implement for our specific Pipeline Handler.

Define a new ``VividCameraPrivate()`` class derived from ``Camera::Private`` by
adding the following code before the PipelineHandlerVivid class definition where
it will be used:

.. code-block:: cpp

   class VividCameraData : public Camera::Private
   {
   public:
          VividCameraData(PipelineHandler *pipe, MediaDevice *media)
                : Camera::Private(pipe), media_(media), video_(nullptr)
          {
          }

          ~VividCameraData()
          {
                delete video_;
          }

          int init();
          void bufferReady(FrameBuffer *buffer);

          MediaDevice *media_;
          V4L2VideoDevice *video_;
          Stream stream_;
   };

This example pipeline handler handles a single video device and supports a
single stream, represented by the ``VividCameraData`` class members. More
complex pipeline handlers might register cameras composed of several video
devices and sub-devices, or multiple streams per camera that represent the
several components of the image capture pipeline. You should represent all these
components in the ``Camera::Private`` derived class when developing a custom
PipelineHandler.

In our example VividCameraData we implement an ``init()`` function to prepare
the object from our PipelineHandler, however the Camera::Private class does not
specify the interface for initialisation and PipelineHandlers can manage this
based on their own needs. Derived Camera::Private classes are used only by their
respective pipeline handlers.

The Camera::Private class stores the context required for each camera instance
and is usually responsible for opening all Devices used in the capture pipeline.

We can now implement the ``init`` function for our example Pipeline Handler to
create a new V4L2 video device from the media entity, which we can specify using
the `MediaDevice::getEntityByName`_ function from the MediaDevice. As our
example is based upon the simplistic Vivid test device, we only need to open a
single capture device named 'vivid-000-vid-cap' by the device.

.. _MediaDevice::getEntityByName: https://libcamera.org/api-html/classlibcamera_1_1MediaDevice.html#ad5d9279329ef4987ceece2694b33e230

.. code-block:: cpp

   int VividCameraData::init()
   {
          video_ = new V4L2VideoDevice(media_->getEntityByName("vivid-000-vid-cap"));
          if (video_->open())
                return -ENODEV;

          return 0;
   }

The VividCameraData should be created and initialised before we move on to
register a new Camera device so we need to construct and initialise our
VividCameraData after we have identified our device within
PipelineHandlerVivid::match(). The VividCameraData is wrapped by a
std::unique_ptr to help manage the lifetime of the instance.

If the camera data initialization fails, return ``false`` to indicate the
failure to the ``match()`` function and prevent retrying of the pipeline
handler.

.. code-block:: cpp

   std::unique_ptr<VividCameraData> data = std::make_unique<VividCameraData>(this, media);

   if (data->init())
           return false;


Once the camera data has been initialized, the Camera device instances and the
associated streams have to be registered. Create a set of streams for the
camera, which for this device is only one. You create a camera using the static
`Camera::create`_ function, passing the Camera::Private instance, the id of the
camera, and the streams available. Then register the camera with the pipeline
handler and camera manager using `registerCamera`_.

Finally with a successful construction, we return 'true' indicating that the
PipelineHandler successfully matched and constructed a device.

.. _Camera::create: https://libcamera.org/api-html/classlibcamera_1_1Camera.html#a453740e0d2a2f495048ae307a85a2574
.. _registerCamera: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#adf02a7f1bbd87aca73c0e8d8e0e6c98b

.. code-block:: cpp

   std::set<Stream *> streams{ &data->stream_ };
   std::shared_ptr<Camera> camera = Camera::create(this, data->video_->deviceName(), streams);
   registerCamera(std::move(camera), std::move(data));

   return true;


Our match function should now look like the following:

.. code-block:: cpp

   bool PipelineHandlerVivid::match(DeviceEnumerator *enumerator)
   {
   	DeviceMatch dm("vivid");
   	dm.add("vivid-000-vid-cap");

   	MediaDevice *media = acquireMediaDevice(enumerator, dm);
   	if (!media)
   		return false;

   	std::unique_ptr<VividCameraData> data = std::make_unique<VividCameraData>(this, media);

   	/* Locate and open the capture video node. */
   	if (data->init())
   		return false;

   	/* Create and register the camera. */
   	std::set<Stream *> streams{ &data->stream_ };
   	const std::string &id = data->video_->deviceName();
   	std::shared_ptr<Camera> camera = Camera::create(data.release(), id, streams);
   	registerCamera(std::move(camera));

   	return true;
   }

We will need to use our custom VividCameraData class frequently throughout the
pipeline handler, so we add a private convenience helper to our Pipeline handler
to obtain and cast the custom VividCameraData instance from a Camera::Private
instance.

.. code-block:: cpp

   private:
       VividCameraData *cameraData(Camera *camera)
       {
               return static_cast<VividCameraData *>(camera->_d());
       }

At this point, you need to add the following new includes to provide the Camera
interface, and device interaction interfaces.

.. code-block:: cpp

   #include <libcamera/camera.h>
   #include "libcamera/internal/media_device.h"
   #include "libcamera/internal/v4l2_videodevice.h"

Registering controls and properties
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The libcamera `controls framework`_ allows an application to configure the
streams capture parameters on a per-frame basis and is also used to advertise
immutable properties of the ``Camera`` device.

The libcamera controls and properties are defined in YAML form which is
processed to automatically generate documentation and interfaces. Controls are
defined by the src/libcamera/`control_ids.yaml`_ file and camera properties
are defined by src/libcamera/`properties_ids.yaml`_.

.. _controls framework: https://libcamera.org/api-html/controls_8h.html
.. _control_ids.yaml: https://libcamera.org/api-html/control__ids_8h.html
.. _properties_ids.yaml: https://libcamera.org/api-html/property__ids_8h.html

Pipeline handlers can optionally register the list of controls an application
can set as well as a list of immutable camera properties. Being both
Camera-specific values, they are represented in the ``Camera::Private`` base
class, which provides two members for this purpose: the
`Camera::Private::controlInfo_`_ and the `Camera::Private::properties_`_ fields.

.. _Camera::Private::controlInfo_: https://libcamera.org/api-html/classlibcamera_1_1Camera_1_1Private.html#ab4e183eb4dabe929d1b2bbbb519b969f
.. _Camera::Private::properties_: https://libcamera.org/api-html/classlibcamera_1_1Camera_1_1Private.html#ad31f12f5ed9c1fbe25750902f4791064

The ``controlInfo_`` field represents a map of ``ControlId`` instances
associated with the limits of valid values supported for the control. More
information can be found in the `ControlInfoMap`_ class documentation.

.. _ControlInfoMap: https://libcamera.org/api-html/classlibcamera_1_1ControlInfoMap.html

Pipeline handlers register controls to expose the tunable device and IPA
parameters to applications. Our example pipeline handler only exposes trivial
controls of the video device, by registering a ``ControlId`` instance with
associated values for each supported V4L2 control but demonstrates the mapping
of V4L2 Controls to libcamera ControlIDs.

Complete the initialization of the ``VividCameraData`` class by adding the
following code to the ``VividCameraData::init()`` function to initialise the
controls. For more complex control configurations, this could of course be
broken out to a separate function, but for now we just initialise the small set
inline in our VividCameraData init:

.. code-block:: cpp

   /* Initialise the supported controls. */
   const ControlInfoMap &controls = video_->controls();
   ControlInfoMap::Map ctrls;

   for (const auto &ctrl : controls) {
           const ControlId *id;
           ControlInfo info;

           switch (ctrl.first->id()) {
           case V4L2_CID_BRIGHTNESS:
                   id = &controls::Brightness;
                   info = ControlInfo{ { -1.0f }, { 1.0f }, { 0.0f } };
                   break;
           case V4L2_CID_CONTRAST:
                   id = &controls::Contrast;
                   info = ControlInfo{ { 0.0f }, { 2.0f }, { 1.0f } };
                   break;
           case V4L2_CID_SATURATION:
                   id = &controls::Saturation;
                   info = ControlInfo{ { 0.0f }, { 2.0f }, { 1.0f } };
                   break;
           default:
                   continue;
           }

           ctrls.emplace(id, info);
   }

   controlInfo_ = std::move(ctrls);

The ``properties_`` field is  a list of ``ControlId`` instances
associated with immutable values, which represent static characteristics that can
be used by applications to identify camera devices in the system. Properties can be
registered by inspecting the values of V4L2 controls from the video devices and
camera sensor (for example to retrieve the position and orientation of a camera)
or to express other immutable characteristics. The example pipeline handler does
not register any property, but examples are available in the libcamera code
base.

.. TODO: Add a property example to the pipeline handler. At least the model.

At this point you need to add the following includes to the top of the file for
handling controls:

.. code-block:: cpp

   #include <libcamera/controls.h>
   #include <libcamera/control_ids.h>

Generating a default configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once ``Camera`` devices and the associated ``Streams`` have been registered, an
application can proceed to acquire and configure the camera to prepare it for a
frame capture session.

Applications specify the requested configuration by assigning a
``StreamConfiguration`` instance to each stream they want to enable which
expresses the desired image size and pixel format. The stream configurations are
grouped in a ``CameraConfiguration`` which is inspected by the pipeline handler
and validated to adjust it to a supported configuration. This may involve
adjusting the formats or image sizes or alignments for example to match the
capabilities of the device.

Applications may choose to repeat validation stages, adjusting parameters until
a set of validated StreamConfigurations are returned that is acceptable for the
applications needs. When the pipeline handler receives a valid camera
configuration it can use the image stream configurations to apply settings to
the hardware devices.

This configuration and validation process is managed with another Pipeline
specific class derived from a common base implementation and interface.

To support validation in our example pipeline handler, Create a new class called
``VividCameraConfiguration`` derived from the base `CameraConfiguration`_ class
which we can implement and use within our ``PipelineHandlerVivid`` class.

.. _CameraConfiguration: https://libcamera.org/api-html/classlibcamera_1_1CameraConfiguration.html

The derived ``CameraConfiguration`` class must override the base class
``validate()`` function, where the stream configuration inspection and
adjustment happens.

.. code-block:: cpp

    class VividCameraConfiguration : public CameraConfiguration
    {
    public:
           VividCameraConfiguration();

           Status validate() override;
    };

    VividCameraConfiguration::VividCameraConfiguration()
           : CameraConfiguration()
    {
    }

Applications generate a ``CameraConfiguration`` instance by calling the
`Camera::generateConfiguration()`_ function, which calls into the pipeline
implementation of the overridden `PipelineHandler::generateConfiguration()`_
function.

.. _Camera::generateConfiguration(): https://libcamera.org/api-html/classlibcamera_1_1Camera.html#a25c80eb7fc9b1cf32692ce0c7f09991d
.. _PipelineHandler::generateConfiguration(): https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a7932e87735695500ce1f8c7ae449b65b

Configurations are generated by receiving a list of ``StreamRoles`` instances,
which libcamera uses as predefined ways an application intends to use a camera
(You can read the full list in the `StreamRole API`_ documentation). These are
optional hints on how an application intends to use a stream, and a pipeline
handler should return an ideal configuration for each role that is requested.

.. _StreamRole API: https://libcamera.org/api-html/stream_8h.html#file_a295d1f5e7828d95c0b0aabc0a8baac03

In the pipeline handler ``generateConfiguration`` implementation, remove the
``return nullptr;``, create a new instance of the ``CameraConfiguration``
derived class, and assign it to a base class pointer.

.. code-block:: cpp

   VividCameraData *data = cameraData(camera);
   CameraConfiguration *config = new VividCameraConfiguration();

A ``CameraConfiguration`` is specific to each pipeline, so you can only create
it from the pipeline handler code path. Applications can also generate an empty
configuration and add desired stream configurations manually. Pipelines must
allow for this by returning an empty configuration if no roles are requested.

To support this in our PipelineHandlerVivid, next add the following check in
``generateConfiguration`` after the Cameraconfiguration has been constructed:

.. code-block:: cpp

   if (roles.empty())
           return config;

A production pipeline handler should generate the ``StreamConfiguration`` for
all the appropriate stream roles a camera device supports. For this simpler
example (with only one stream), the pipeline handler always returns the same
configuration, inferred from the underlying V4L2VideoDevice.

How it does this is shown below, but examination of the more full-featured
pipelines for IPU3, RKISP1 and RaspberryPi are recommended to explore more
complex examples.

To generate a ``StreamConfiguration``, you need a list of pixel formats and
frame sizes which are supported as outputs of the stream. You can fetch a map of
the ``V4LPixelFormat`` and ``SizeRange`` supported by the underlying output
device, but the pipeline handler needs to convert this to a
``libcamera::PixelFormat`` type to pass to applications. We do this here using
``std::transform`` to convert the formats and populate a new ``PixelFormat`` map
as shown below.

Continue adding the following code example to our ``generateConfiguration``
implementation.

.. code-block:: cpp

   std::map<V4L2PixelFormat, std::vector<SizeRange>> v4l2Formats =
           data->video_->formats();
   std::map<PixelFormat, std::vector<SizeRange>> deviceFormats;
   std::transform(v4l2Formats.begin(), v4l2Formats.end(),
          std::inserter(deviceFormats, deviceFormats.begin()),
          [&](const decltype(v4l2Formats)::value_type &format) {
              return decltype(deviceFormats)::value_type{
                  format.first.toPixelFormat(),
                  format.second
              };
          });

The `StreamFormats`_ class holds information about the pixel formats and frame
sizes that a stream can support. The class groups size information by the pixel
format, which can produce it.

.. _StreamFormats: https://libcamera.org/api-html/classlibcamera_1_1StreamFormats.html

The code below uses the ``StreamFormats`` class to represent all of the
supported pixel formats, associated with a list of frame sizes. It then
generates a supported StreamConfiguration to model the information an
application can use to configure a single stream.

Continue adding the following code to support this:

.. code-block:: cpp

   StreamFormats formats(deviceFormats);
   StreamConfiguration cfg(formats);

As well as a list of supported StreamFormats, the StreamConfiguration is also
expected to provide an initialised default configuration. This may be arbitrary,
but depending on use case you may wish to select an output that matches the
Sensor output, or prefer a pixelformat which might provide higher performance on
the hardware. The bufferCount represents the number of buffers required to
support functional continuous processing on this stream.

.. code-block:: cpp

   cfg.pixelFormat = formats::BGR888;
   cfg.size = { 1280, 720 };
   cfg.bufferCount = 4;

Finally add each ``StreamConfiguration`` generated to the
``CameraConfiguration``, and ensure that it has been validated before returning
it to the application. With only a single supported stream, this code adds only
a single StreamConfiguration. However a StreamConfiguration should be added for
each supported role in a device that can handle more streams.

Add the following code to complete the implementation of
``generateConfiguration``:

.. code-block:: cpp

   config->addConfiguration(cfg);

   config->validate();

   return config;

To validate a camera configuration, a pipeline handler must implement the
`CameraConfiguration::validate()`_ function in its derived class to inspect all
the stream configuration associated to it, make any adjustments required to make
the configuration valid, and return the validation status.

If changes are made, it marks the configuration as ``Adjusted``, however if the
requested configuration is not supported and cannot be adjusted it shall be
refused and marked as ``Invalid``.

.. _CameraConfiguration::validate(): https://libcamera.org/api-html/classlibcamera_1_1CameraConfiguration.html#a29f8f263384c6149775b6011c7397093

The validation phase makes sure all the platform-specific constraints are
respected by the requested configuration. The most trivial examples being making
sure the requested image formats are supported and the image alignment
restrictions adhered to. The pipeline handler specific implementation of
``validate()`` shall inspect all the configuration parameters received and never
assume they are correct, as applications are free to change the requested stream
parameters after the configuration has been generated.

Again, this example pipeline handler is simpler, look at the more complex
implementations for a realistic example.

Add the following function implementation to your file:

.. code-block:: cpp

   CameraConfiguration::Status VividCameraConfiguration::validate()
   {
           Status status = Valid;

           if (config_.empty())
                  return Invalid;

           if (config_.size() > 1) {
                  config_.resize(1);
                  status = Adjusted;
           }

           StreamConfiguration &cfg = config_[0];

           const std::vector<libcamera::PixelFormat> formats = cfg.formats().pixelformats();
           if (std::find(formats.begin(), formats.end(), cfg.pixelFormat) == formats.end()) {
                  cfg.pixelFormat = cfg.formats().pixelformats()[0];
                  LOG(VIVID, Debug) << "Adjusting format to " << cfg.pixelFormat.toString();
                  status = Adjusted;
           }

           cfg.bufferCount = 4;

           return status;
   }

Now that we are handling the ``PixelFormat`` type, we also need to add
``#include <libcamera/formats.h>`` to the include section before we rebuild the
codebase, and test:

.. code-block:: shell

   ninja -C build
   LIBCAMERA_LOG_LEVELS=Pipeline,VIVID:0 ./build/src/cam/cam -c vivid -I

You should see the following output showing the capabilites of our new pipeline
handler, and showing that our configurations have been generated:

.. code-block:: shell

    Using camera vivid
    0: 1280x720-BGR888
    * Pixelformat: NV21 (320x180)-(3840x2160)/(+0,+0)
    - 320x180
    - 640x360
    - 640x480
    - 1280x720
    - 1920x1080
    - 3840x2160
    * Pixelformat: NV12 (320x180)-(3840x2160)/(+0,+0)
    - 320x180
    - 640x360
    - 640x480
    - 1280x720
    - 1920x1080
    - 3840x2160
    * Pixelformat: BGRA8888 (320x180)-(3840x2160)/(+0,+0)
    - 320x180
    - 640x360
    - 640x480
    - 1280x720
    - 1920x1080
    - 3840x2160
    * Pixelformat: RGBA8888 (320x180)-(3840x2160)/(+0,+0)
    - 320x180
    - 640x360
    - 640x480
    - 1280x720
    - 1920x1080
    - 3840x2160

Configuring a device
~~~~~~~~~~~~~~~~~~~~

With the configuration generated, and optionally modified and re-validated, a
pipeline handler needs a function that allows an application to apply a
configuration to the hardware devices.

The `PipelineHandler::configure()`_ function receives a valid
`CameraConfiguration`_ and applies the settings to hardware devices, using its
parameters to prepare a device for a streaming session with the desired
properties.

.. _PipelineHandler::configure(): https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a930f2a9cdfb51dfb4b9ca3824e84fc29
.. _CameraConfiguration: https://libcamera.org/api-html/classlibcamera_1_1CameraConfiguration.html

Replace the contents of the stubbed ``PipelineHandlerVivid::configure`` function
with the following to obtain the camera data and stream configuration. This
pipeline handler supports only a single stream, so it directly obtains the first
``StreamConfiguration`` from the camera configuration. A pipeline handler with
multiple streams should inspect each StreamConfiguration and configure the
system accordingly.

.. code-block:: cpp

   VividCameraData *data = cameraData(camera);
   StreamConfiguration &cfg = config->at(0);
   int ret;

The Vivid capture device is a V4L2 video device, so we use a `V4L2DeviceFormat`_
with the fourcc and size attributes to apply directly to the capture device
node. The fourcc attribute is a `V4L2PixelFormat`_ and differs from the
``libcamera::PixelFormat``. Converting the format requires knowledge of the
plane configuration for multiplanar formats, so you must explicitly convert it
using the helper ``V4L2VideoDevice::toV4L2PixelFormat()`` provided by the
V4L2VideoDevice instance that the format will be applied on.

.. _V4L2DeviceFormat: https://libcamera.org/api-html/classlibcamera_1_1V4L2DeviceFormat.html
.. _V4L2PixelFormat: https://libcamera.org/api-html/classlibcamera_1_1V4L2PixelFormat.html

Add the following code beneath the code from above:

.. code-block:: cpp

   V4L2DeviceFormat format = {};
   format.fourcc = data->video_->toV4L2PixelFormat(cfg.pixelFormat);
   format.size = cfg.size;

Set the video device format defined above using the
`V4L2VideoDevice::setFormat()`_ function. You should check if the kernel
driver has adjusted the format, as this shows the pipeline handler has failed to
handle the validation stages correctly, and the configure operation shall also
fail.

.. _V4L2VideoDevice::setFormat(): https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#ad67b47dd9327ce5df43350b80c083cca

Continue the implementation with the following code:

.. code-block:: cpp

   ret = data->video_->setFormat(&format);
   if (ret)
          return ret;

   if (format.size != cfg.size ||
          format.fourcc != data->video_->toV4L2PixelFormat(cfg.pixelFormat))
          return -EINVAL;

Finally, store and set stream-specific data reflecting the state of the stream.
Associate the configuration with the stream by using the
`StreamConfiguration::setStream`_ function, and set the values of individual
stream configuration members as required.

.. _StreamConfiguration::setStream: https://libcamera.org/api-html/structlibcamera_1_1StreamConfiguration.html#a74a0eb44dad1b00112c7c0443ae54a12

.. NOTE: the cfg.setStream() call here associates the stream to the
   StreamConfiguration however that should quite likely be done as part of
   the validation process. TBD

Complete the configure implementation with the following code:

.. code-block:: cpp

   cfg.setStream(&data->stream_);
   cfg.stride = format.planes[0].bpl;

   return 0;

.. TODO: stride SHALL be assigned in validate

Initializing device controls
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Pipeline handlers can optionally initialize the video devices and camera sensor
controls at system configuration time, to make sure they are defaulted to sane
values. Handling of device controls is again performed using the libcamera
`controls framework`_.

.. _Controls Framework: https://libcamera.org/api-html/controls_8h.html

This section is particularly specific to Vivid as it sets the initial values of
controls to match `Vivid Controls`_ defined by the kernel driver. You won't need
any of the code below for your pipeline handler, but it's included as an example
of how to implement functionality your pipeline handler might need.

.. _Vivid Controls: https://www.kernel.org/doc/html/latest/admin-guide/media/vivid.html#controls

We need to add some definitions at the top of the file for convenience. These
come directly from the kernel sources:

.. code-block:: cpp

   #define VIVID_CID_VIVID_BASE            (0x00f00000 | 0xf000)
   #define VIVID_CID_VIVID_CLASS           (0x00f00000 | 1)
   #define VIVID_CID_TEST_PATTERN          (VIVID_CID_VIVID_BASE  + 0)
   #define VIVID_CID_OSD_TEXT_MODE         (VIVID_CID_VIVID_BASE  + 1)
   #define VIVID_CID_HOR_MOVEMENT          (VIVID_CID_VIVID_BASE  + 2)

We can now use the V4L2 control IDs to prepare a list of controls with the
`ControlList`_ class, and set them using the `ControlList::set()`_ function.

.. _ControlList: https://libcamera.org/api-html/classlibcamera_1_1ControlList.html
.. _ControlList::set(): https://libcamera.org/api-html/classlibcamera_1_1ControlList.html#a74a1a29abff5243e6e37ace8e24eb4ba

In our pipeline ``configure`` function, add the following code after the format
has been set and checked to initialise the ControlList and apply it to the
device:

.. code-block:: cpp

   ControlList controls(data->video_->controls());
   controls.set(VIVID_CID_TEST_PATTERN, 0);
   controls.set(VIVID_CID_OSD_TEXT_MODE, 0);

   controls.set(V4L2_CID_BRIGHTNESS, 128);
   controls.set(V4L2_CID_CONTRAST, 128);
   controls.set(V4L2_CID_SATURATION, 128);

   controls.set(VIVID_CID_HOR_MOVEMENT, 5);

   ret = data->video_->setControls(&controls);
   if (ret) {
          LOG(VIVID, Error) << "Failed to set controls: " << ret;
          return ret < 0 ? ret : -EINVAL;
   }

These controls configure VIVID to use a default test pattern, and enable all
on-screen display text, while configuring sensible brightness, contrast and
saturation values. Use the ``controls.set`` function to set individual controls.

Buffer handling and stream control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once the system has been configured with the requested parameters, it is now
possible for applications to start capturing frames from the ``Camera`` device.

libcamera implements a per-frame request capture model, realized by queueing
``Request`` instances to a ``Camera`` object. Before applications can start
submitting capture requests the capture pipeline needs to be prepared to deliver
frames as soon as they are requested. Memory should be initialized and made
available to the devices which have to be started and ready to produce
images. At the end of a capture session the ``Camera`` device needs to be
stopped, to gracefully clean up any allocated memory and stop the hardware
devices. Pipeline handlers implement two functions for these purposes, the
``start()`` and ``stop()`` functions.

The memory initialization phase that happens at ``start()`` time serves to
configure video devices to be able to use memory buffers exported as dma-buf
file descriptors. From the pipeline handlers perspective the video devices that
provide application facing streams always act as memory importers which use,
in V4L2 terminology, buffers of V4L2_MEMORY_DMABUF memory type.

libcamera also provides an API to allocate and export memory to applications
realized through the `exportFrameBuffers`_ function and the
`FrameBufferAllocator`_ class which will be presented later.

.. _exportFrameBuffers: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a6312a69da7129c2ed41f9d9f790adf7c
.. _FrameBufferAllocator: https://libcamera.org/api-html/classlibcamera_1_1FrameBufferAllocator.html

Please refer to the V4L2VideoDevice API documentation, specifically the
`allocateBuffers`_, `importBuffers`_ and `exportBuffers`_ functions for a
detailed description of the video device memory management.

.. _allocateBuffers: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#a3a1a77e5e6c220ea7878e89485864a1c
.. _importBuffers: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#a154f5283d16ebd5e15d63e212745cb64
.. _exportBuffers: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#ae9c0b0a68f350725b63b73a6da5a2ecd

Video memory buffers are represented in libcamera by the `FrameBuffer`_ class.
A ``FrameBuffer`` instance has to be associated to each ``Stream`` which is part
of a capture ``Request``. Pipeline handlers should prepare the capture devices
by importing the dma-buf file descriptors it needs to operate on. This operation
is performed by using the ``V4L2VideoDevice`` API, which provides an
``importBuffers()`` function that prepares the video device accordingly.

.. _FrameBuffer: https://libcamera.org/api-html/classlibcamera_1_1FrameBuffer.html

Implement the pipeline handler ``start()`` function by replacing the stub
version with the following code:

.. code-block:: c++

   VividCameraData *data = cameraData(camera);
   unsigned int count = data->stream_.configuration().bufferCount;

   int ret = data->video_->importBuffers(count);
   if (ret < 0)
         return ret;

   return 0;

During the startup phase pipeline handlers allocate any internal buffer pool
required to transfer data between different components of the image capture
pipeline, for example, between the CSI-2 receiver and the ISP input. The example
pipeline does not require any internal pool, but examples are available in more
complex pipeline handlers in the libcamera code base.

Applications might want to use memory allocated in the video devices instead of
allocating it from other parts of the system. libcamera provides an abstraction
to assist with this task in the `FrameBufferAllocator`_ class. The
``FrameBufferAllocator`` reserves memory for a ``Stream`` in the video device
and exports it as dma-buf file descriptors. From this point on, the allocated
``FrameBuffer`` are associated to ``Stream`` instances in a ``Request`` and then
imported by the pipeline hander in exactly the same fashion as if they were
allocated elsewhere.

.. _FrameBufferAllocator: https://libcamera.org/api-html/classlibcamera_1_1FrameBufferAllocator.html

Pipeline handlers support the ``FrameBufferAllocator`` operations by
implementing the `exportFrameBuffers`_ function, which will allocate memory in
the video device associated with a stream and export it.

.. _exportFrameBuffers: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a6312a69da7129c2ed41f9d9f790adf7c

Implement the ``exportFrameBuffers`` stub function with the following code to
handle this:

.. code-block:: cpp

   unsigned int count = stream->configuration().bufferCount;
   VividCameraData *data = cameraData(camera);

   return data->video_->exportBuffers(count, buffers);

Once memory has been properly setup, the video devices can be started, to
prepare for capture operations. Complete the ``start`` function implementation
with the following code:

.. code-block:: cpp

   ret = data->video_->streamOn();
   if (ret < 0) {
          data->video_->releaseBuffers();
          return ret;
   }

   return 0;

The function starts the video device associated with the stream with the
`streamOn`_ function. If the call fails, the error value is propagated to the
caller and the `releaseBuffers`_ function releases any buffers to leave the
device in a consistent state. If your pipeline handler uses any image processing
algorithms, or other devices you should also stop them.

.. _streamOn: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#a588a5dc9d6f4c54c61136ac43ff9a8cc
.. _releaseBuffers: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#a191619c152f764e03bc461611f3fcd35

Of course we also need to handle the corresponding actions to stop streaming on
a device, Add the following to the ``stop`` function, to stop the stream with
the `streamOff`_ function and release all buffers.

.. _streamOff: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#a61998710615bdf7aa25a046c8565ed66

.. code-block:: cpp

   VividCameraData *data = cameraData(camera);
   data->video_->streamOff();
   data->video_->releaseBuffers();

Queuing requests between applications and hardware
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

libcamera implements a streaming model based on capture requests queued by an
application to the ``Camera`` device. Each request contains at least one
``Stream`` instance with an associated ``FrameBuffer`` object.

When an application sends a capture request, the pipeline handler identifies
which video devices have to be provided with buffers to generate a frame from
the enabled streams.

This example pipeline handler identifies the buffer using the `findBuffer`_
helper from the only supported stream and queues it to the capture device
directly with the `queueBuffer`_ function provided by the V4L2VideoDevice.

.. _findBuffer: https://libcamera.org/api-html/classlibcamera_1_1Request.html#ac66050aeb9b92c64218945158559c4d4
.. _queueBuffer: https://libcamera.org/api-html/classlibcamera_1_1V4L2VideoDevice.html#a594cd594686a8c1cf9ae8dba0b2a8a75

Replace the stubbed contents of ``queueRequestDevice`` with the following:

.. code-block:: cpp

   VividCameraData *data = cameraData(camera);
   FrameBuffer *buffer = request->findBuffer(&data->stream_);
   if (!buffer) {
          LOG(VIVID, Error)
                  << "Attempt to queue request with invalid stream";

          return -ENOENT;
   }

   int ret = data->video_->queueBuffer(buffer);
   if (ret < 0)
          return ret;

   return 0;

Processing controls
~~~~~~~~~~~~~~~~~~~

Capture requests not only contain streams and memory buffers, but can
optionally contain a list of controls the application has set to modify the
streaming parameters.

Applications can set controls registered by the pipeline handler in the
initialization phase, as explained in the `Registering controls and properties`_
section.

Implement a ``processControls`` function above the ``queueRequestDevice``
function to loop through the control list received with each request, and
inspect the control values. Controls may need to be converted between the
libcamera control range definitions and their corresponding values on the device
before being set.

.. code-block:: cpp

   int PipelineHandlerVivid::processControls(VividCameraData *data, Request *request)
   {
          ControlList controls(data->video_->controls());

          for (auto it : request->controls()) {
                 unsigned int id = it.first;
                 unsigned int offset;
                 uint32_t cid;

                 if (id == controls::Brightness) {
                        cid = V4L2_CID_BRIGHTNESS;
                        offset = 128;
                 } else if (id == controls::Contrast) {
                        cid = V4L2_CID_CONTRAST;
                        offset = 0;
                 } else if (id == controls::Saturation) {
                        cid = V4L2_CID_SATURATION;
                        offset = 0;
                 } else {
                        continue;
                 }

                 int32_t value = lroundf(it.second.get<float>() * 128 + offset);
                 controls.set(cid, std::clamp(value, 0, 255));
          }

          for (const auto &ctrl : controls)
                 LOG(VIVID, Debug)
                        << "Setting control " << utils::hex(ctrl.first)
                        << " to " << ctrl.second.toString();

          int ret = data->video_->setControls(&controls);
          if (ret) {
                 LOG(VIVID, Error) << "Failed to set controls: " << ret;
                 return ret < 0 ? ret : -EINVAL;
          }

          return ret;
   }

Declare the function prototype for the ``processControls`` function within the
private ``PipelineHandlerVivid`` class members, as it is only used internally as
a helper when processing Requests.

.. code-block:: cpp

   private:
        int processControls(VividCameraData *data, Request *request);

A pipeline handler is responsible for applying controls provided in a Request to
the relevant hardware devices. This could be directly on the capture device, or
where appropriate by setting controls on V4L2Subdevices directly. Each pipeline
handler is responsible for understanding the correct procedure for applying
controls to the device they support.

This example pipeline handler applies controls during the `queueRequestDevice`_
function for each request, and applies them to the capture device through the
capture node.

.. _queueRequestDevice: https://libcamera.org/api-html/classlibcamera_1_1PipelineHandler.html#a106914cca210640c9da9ee1f0419e83c

In the ``queueRequestDevice`` function, replace the following:

.. code-block:: cpp

   int ret = data->video_->queueBuffer(buffer);
   if (ret < 0)
        return ret;

With the following code:

.. code-block:: cpp

   int ret = processControls(data, request);
   if (ret < 0)
        return ret;

   ret = data->video_->queueBuffer(buffer);
   if (ret < 0)
        return ret;

We also need to add the following include directive to support the control
value translation operations:

.. code-block:: cpp

   #include <math.h>

Frame completion and event handling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

libcamera implements a signals and slots mechanism (similar to `Qt Signals and
Slots`_) to connect event sources with callbacks to handle them.

As a general summary, a ``Slot`` can be connected to a ``Signal``, which when
emitted triggers the execution of the connected slots.  A detailed description
of the libcamera implementation is available in the `libcamera Signal and Slot`_
classes documentation.

.. _Qt Signals and Slots: https://doc.qt.io/qt-5/signalsandslots.html
.. _libcamera Signal and Slot: https://libcamera.org/api-html/classlibcamera_1_1Signal.html#details

In order to notify applications about the availability of new frames and data,
the ``Camera`` device exposes two ``Signals`` to which applications can connect
to be notified of frame completion events. The ``bufferComplete`` signal serves
to report to applications the completion event of a single ``Stream`` part of a
``Request``, while the ``requestComplete`` signal notifies the completion of all
the ``Streams`` and data submitted as part of a request. This mechanism allows
implementation of partial request completion, which allows an application to
inspect completed buffers associated with the single streams without waiting for
all of them to be ready.

The ``bufferComplete`` and ``requestComplete`` signals are emitted by the
``Camera`` device upon notifications received from the pipeline handler, which
tracks the buffers and request completion status.

The single buffer completion notification is implemented by pipeline handlers by
`connecting`_ the ``bufferReady`` signal of the capture devices they have queued
buffers to, to a member function slot that handles processing of the completed
frames. When a buffer is ready, the pipeline handler must propagate the
completion of that buffer to the Camera by using the PipelineHandler base class
``completeBuffer`` function. When all of the buffers referenced by a ``Request``
have been completed, the pipeline handler must again notify the ``Camera`` using
the PipelineHandler base class ``completeRequest`` function. The PipelineHandler
class implementation makes sure the request completion notifications are
delivered to applications in the same order as they have been submitted.

.. _connecting: https://libcamera.org/api-html/classlibcamera_1_1Signal.html#aa04db72d5b3091ffbb4920565aeed382

Returning to the ``int VividCameraData::init()`` function, add the following
above the closing ``return 0;`` to connect the pipeline handler ``bufferReady``
function to the V4L2 device buffer signal.

.. code-block:: cpp

   video_->bufferReady.connect(this, &VividCameraData::bufferReady);

Create the matching ``VividCameraData::bufferReady`` function after your
VividCameradata::init() impelementation.

The ``bufferReady`` function obtains the request from the buffer using the
``request`` function, and notifies the ``Camera`` that the buffer and
request are completed. In this simpler pipeline handler, there is only one
stream, so it completes the request immediately. You can find a more complex
example of event handling with supporting multiple streams in the libcamera
code-base.

.. TODO: Add link

.. code-block:: cpp

   void VividCameraData::bufferReady(FrameBuffer *buffer)
   {
          Request *request = buffer->request();

          pipe_->completeBuffer(request, buffer);
          pipe_->completeRequest(request);
   }

Testing a pipeline handler
~~~~~~~~~~~~~~~~~~~~~~~~~~

Once you've built the pipeline handler, we can rebuild the code base, and test
capture through the pipeline through both of the cam and qcam utilities.

.. code-block:: shell

   ninja -C build
   ./build/src/cam/cam -c vivid -C5

To test that the pipeline handler can detect a device, and capture input.

Running the command above outputs (a lot of) information about pixel formats,
and then starts capturing frame data, and should provide an output such as the
following:

.. code-block:: none

   user@dev:/home/libcamera$ ./build/src/cam/cam -c vivid -C5
   [42:34:08.573066847] [186470]  INFO IPAManager ipa_manager.cpp:136 libcamera is not installed. Adding '/home/libcamera/build/src/ipa' to the IPA search path
   [42:34:08.575908115] [186470]  INFO Camera camera_manager.cpp:287 libcamera v0.0.11+876-7b27d262
   [42:34:08.610334268] [186471]  INFO IPAProxy ipa_proxy.cpp:122 libcamera is not installed. Loading IPA configuration from '/home/libcamera/src/ipa/vimc/data'
   Using camera vivid
   [42:34:08.618462130] [186470]  WARN V4L2 v4l2_pixelformat.cpp:176 Unsupported V4L2 pixel format Y10
   ... <remaining Unsupported V4L2 pixel format warnings can be ignored>
   [42:34:08.619901297] [186470]  INFO Camera camera.cpp:793 configuring streams: (0) 1280x720-BGR888
   Capture 5 frames
   fps: 0.00 stream0 seq: 000000 bytesused: 2764800
   fps: 4.98 stream0 seq: 000001 bytesused: 2764800
   fps: 5.00 stream0 seq: 000002 bytesused: 2764800
   fps: 5.03 stream0 seq: 000003 bytesused: 2764800
   fps: 5.03 stream0 seq: 000004 bytesused: 2764800

This demonstrates that the pipeline handler is successfully capturing frames,
but it is helpful to see the visual output and validate the images are being
processed correctly. The libcamera project also implements a Qt based
application which will render the frames in a window for visual inspection:

.. code-block:: shell

   ./build/src/qcam/qcam -c vivid

.. TODO: Running qcam with the vivid pipeline handler appears to have a bug and
         no visual frames are seen. However disabling zero-copy on qcam renders
         them successfully.
