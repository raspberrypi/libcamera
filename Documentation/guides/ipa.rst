.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: ../documentation-contents.rst

IPA Writer's Guide
==================

IPA modules are Image Processing Algorithm modules. They provide functionality
that the pipeline handler can use for image processing.

This guide covers the definition of the IPA interface, and how to plumb the
connection between the pipeline handler and the IPA.

The IPA interface and protocol
------------------------------

The IPA interface defines the interface between the pipeline handler and the
IPA. Specifically, it defines the functions that the IPA exposes that the
pipeline handler can call, and the signals that the pipeline handler can
connect to, in order to receive data from the IPA asynchronously. In addition,
it contains any custom data structures that the pipeline handler and IPA may
pass to each other.

It is possible to use the same IPA interface with multiple pipeline handlers
on different hardware platforms. Generally in such cases, these platforms would
have a common hardware ISP pipeline. For instance, the rkisp1 pipeline handler
supports both the RK3399 and the i.MX8MP as they integrate the same ISP.
However, the i.MX8MP has a more complex camera pipeline, which may call for a
dedicated pipeline handler in the future. As the ISP is the same as for RK3399,
the same IPA interface could be used for both pipeline handlers. The build files
provide a mapping from pipeline handler to the IPA interface name as detailed in
:ref:`compiling-section`.

The IPA protocol refers to the agreement between the pipeline handler and the
IPA regarding the expected response(s) from the IPA for given calls to the IPA.
This protocol doesn't need to be declared anywhere in code, but it shall be
documented, as there may be multiple IPA implementations for one pipeline
handler.

As part of the design of libcamera, IPAs may be isolated in a separate process,
or run in the same process but a different thread from libcamera. The pipeline
handler and IPA shall not have to change their operation based on whether the
IPA is isolated or not, but the possibility of isolation needs to be kept in
mind. Therefore all data that is passed between them must be serializable, so
they must be defined separately in the `mojo Interface Definition Language`_
(IDL), and a code generator will generate headers and serializers corresponding
to the definitions. Every interface is defined in a mojom file and includes:

- the functions that the pipeline handler can call from the IPA
- signals in the pipeline handler that the IPA can emit
- any data structures that are to be passed between the pipeline handler and the IPA

All IPA modules of a given pipeline handler use the same IPA interface. The IPA
interface definition is thus written by the pipeline handler author, based on
how they design the interactions between the pipeline handler and the IPA.

The entire IPA interface, including the functions, signals, and any custom
structs shall be defined in a file named {interface_name}.mojom under
include/libcamera/ipa/.

.. _mojo Interface Definition Language: https://chromium.googlesource.com/chromium/src.git/+/master/mojo/public/tools/bindings/README.md

Namespacing
-----------

To avoid name collisions between data types defined by different IPA interfaces
and data types defined by libcamera, each IPA interface must be defined in its
own namespace.

The namespace is specific with mojo's module directive. It must be the first
non-comment line in the mojo data definition file. For example, the Raspberry
Pi IPA interface uses:

.. code-block:: none

        module ipa.rpi;

This will become the ipa::rpi namespace in C++ code.

Data containers
---------------

Since the data passed between the pipeline handler and the IPA must support
serialization, any custom data containers must be defined with the mojo IDL.

The following list of libcamera objects are supported in the interface
definition, and may be used as function parameter types or struct field types:

- libcamera.ControlInfoMap
- libcamera.ControlList
- libcamera.FileDescriptor
- libcamera.IPABuffer
- libcamera.IPACameraSensorInfo
- libcamera.IPASettings
- libcamera.IPAStream
- libcamera.Point
- libcamera.Rectangle
- libcamera.Size
- libcamera.SizeRange

To use them, core.mojom must be included in the mojo data definition file:

.. code-block:: none

        import "include/libcamera/ipa/core.mojom";

Other custom structs may be defined and used as well. There is no requirement
that they must be defined before usage. enums and structs are supported.

The following is an example of a definition of an enum, for the purpose of
being used as flags:

.. code-block:: none

        enum ConfigParameters {
                ConfigLsTable = 0x01,
                ConfigStaggeredWrite = 0x02,
                ConfigSensor = 0x04,
                ConfigDropFrames = 0x08,
        };

The following is an example of a definition of a struct:

.. code-block:: none

        struct ConfigInput {
                uint32 op;
                uint32 transform;
                libcamera.FileDescriptor lsTableHandle;
                int32 lsTableHandleStatic = -1;
                map<uint32, libcamera.IPAStream> streamConfig;
                array<libcamera.IPABuffer> buffers;
        };

This example has some special things about it. First of all, it uses the
FileDescriptor data type. This type must be used to ensure that the file
descriptor that it contains is translated properly across the IPC boundary
(when the IPA is in an isolated process).

This does mean that if the file descriptor should be sent without being
translated (for example, for the IPA to tell the pipeline handler which
fd *that the pipeline handler holds* to act on), then it must be in a
regular int32 type.

This example also illustrates that struct fields may have default values, as
is assigned to lsTableHandleStatic. This is the value that the field will
take when the struct is constructed with the default constructor.

Arrays and maps are supported as well. They are translated to C++ vectors and
maps, respectively. The members of the arrays and maps are embedded, and cannot
be const.

Note that nullable fields, static-length arrays, handles, and unions, which
are supported by mojo, are not supported by our code generator.

The Main IPA interface
----------------------

The IPA interface is split in two parts, the Main IPA interface, which
describes the functions that the pipeline handler can call from the IPA,
and the Event IPA interface, which describes the signals received by the
pipeline handler that the IPA can emit. Both must be defined. This section
focuses on the Main IPA interface.

The main interface must be named as IPA{interface_name}Interface.

The functions that the pipeline handler can call from the IPA may be
synchronous or asynchronous. Synchronous functions do not return until the IPA
returns from the function, while asynchronous functions return immediately
without waiting for the IPA to return.

At a minimum, the following three functions must be present (and implemented):

- init();
- start();
- stop();

All three of these functions are synchronous. The parameters for start() and
init() may be customized.

init() initializes the IPA interface. It shall be called before any other
function of the IPAInterface.

stop() informs the IPA module that the camera is stopped. The IPA module shall
release resources prepared in start().

A configure() function is recommended. Any ControlInfoMap instances that will be
used by the IPA must be sent to the IPA from the pipeline handler, at configure
time, for example.

All input parameters will become const references, except for arithmetic types,
which will be passed by value. Output parameters will become pointers, unless
the first output parameter is an int32, or there is only one primitive output
parameter, in which case it will become a regular return value.

const is not allowed inside of arrays and maps. mojo arrays will become C++
std::vector<>.

By default, all functions defined in the main interface are synchronous. This
means that in the case of IPC (i.e. isolated IPA), the function call will not
return until the return value or output parameters are ready. To specify an
asynchronous function, the [async] attribute can be used. Asynchronous
functions must not have any return value or output parameters, since in the
case of IPC the call needs to return immediately.

It is also possible that the IPA will not be run in isolation. In this case,
the IPA thread will not exist until start() is called. This means that in the
case of no isolation, asynchronous calls cannot be made before start(). Since
the IPA interface must be the same regardless of isolation, the same
restriction applies to the case of isolation, and any function that will be
called before start() must be synchronous.

In addition, any call made after start() and before stop() must be
asynchronous. The motivation for this is to avoid damaging real-time
performance of the pipeline handler. If the pipeline handler wants some data
from the IPA, the IPA should return the data asynchronously via an event
(see "The Event IPA interface").

The following is an example of a main interface definition:

.. code-block:: none

        interface IPARPiInterface {
                init(libcamera.IPASettings settings, string sensorName)
                        => (int32 ret, bool metadataSupport);
                start() => (int32 ret);
                stop();

                configure(libcamera.IPACameraSensorInfo sensorInfo,
                          map<uint32, libcamera.IPAStream> streamConfig,
                          map<uint32, libcamera.ControlInfoMap> entityControls,
                          ConfigInput ipaConfig)
                        => (int32 ret, ConfigOutput results);

                mapBuffers(array<IPABuffer> buffers);
                unmapBuffers(array<uint32> ids);

                [async] signalStatReady(uint32 bufferId);
                [async] signalQueueRequest(libcamera.ControlList controls);
                [async] signalIspPrepare(ISPConfig data);
        };


The first three functions are the required functions. Functions do not need to
have return values, like stop(), mapBuffers(), and unmapBuffers(). In the case
of asynchronous functions, as explained before, they *must not* have return
values.

The Event IPA interface
-----------------------

The event IPA interface describes the signals received by the pipeline handler
that the IPA can emit. It must be defined. If there are no event functions,
then it may be empty. These emissions are meant to notify the pipeline handler
of some event, such as request data is ready, and *must not* be used to drive
the camera pipeline from the IPA.

The event interface must be named as IPA{interface_name}EventInterface.

Functions defined in the event interface are implicitly asynchronous.
Thus they cannot return any value. Specifying the [async] tag is not
necessary.

Functions defined in the event interface will become signals in the IPA
interface. The IPA can emit signals, while the pipeline handler can connect
slots to them.

The following is an example of an event interface definition:

.. code-block:: none

        interface IPARPiEventInterface {
                statsMetadataComplete(uint32 bufferId,
                                      libcamera.ControlList controls);
                runIsp(uint32 bufferId);
                embeddedComplete(uint32 bufferId);
                setIsp(libcamera.ControlList controls);
                setStaggered(libcamera.ControlList controls);
        };

.. _compiling-section:

Compiling the IPA interface
---------------------------

After the IPA interface is defined in include/libcamera/ipa/{interface_name}.mojom,
an entry for it must be added in meson so that it can be compiled. The filename
must be added to the pipeline_ipa_mojom_mapping variable in
include/libcamera/ipa/meson.build. This variable maps the pipeline handler name
to its IPA interface file.

For example, adding the raspberrypi.mojom file to meson:

.. code-block:: none

        pipeline_ipa_mojom_mapping = [
            'rpi/vc4': 'raspberrypi.mojom',
        ]

This will cause the mojo data definition file to be compiled. Specifically, it
generates five files:

- a header describing the custom data structures, and the complete IPA
  interface (at {$build_dir}/include/libcamera/ipa/{interface}_ipa_interface.h)

- a serializer implementing de/serialization for the custom data structures (at
  {$build_dir}/include/libcamera/ipa/{interface}_ipa_serializer.h)

- a proxy header describing a specialized IPA proxy (at
  {$build_dir}/include/libcamera/ipa/{interface}_ipa_proxy.h)

- a proxy source implementing the IPA proxy (at
  {$build_dir}/src/libcamera/proxy/{interface}_ipa_proxy.cpp)

- a proxy worker source implementing the other end of the IPA proxy (at
  {$build_dir}/src/libcamera/proxy/worker/{interface}_ipa_proxy_worker.cpp)

The IPA proxy serves as the layer between the pipeline handler and the IPA, and
handles threading vs isolation transparently. The pipeline handler and the IPA
only require the interface header and the proxy header. The serializer is only
used internally by the proxy.

Using the custom data structures
--------------------------------

To use the custom data structures that are defined in the mojo data definition
file, the following header must be included:

.. code-block:: C++

   #include <libcamera/ipa/{interface_name}_ipa_interface.h>

The POD types of the structs simply become their C++ counterparts, eg. uint32
in mojo will become uint32_t in C++. mojo map becomes C++ std::map, and mojo
array becomes C++ std::vector. All members of maps and vectors are embedded,
and are not pointers. The members cannot be const.

The names of all the fields of structs can be used in C++ in exactly the same
way as they are defined in the data definition file. For example, the following
struct as defined in the mojo file:

.. code-block:: none

   struct SensorConfig {
        uint32 gainDelay = 1;
        uint32 exposureDelay;
        uint32 sensorMetadata;
   };

Will become this in C++:

.. code-block:: C++

   struct SensorConfig {
        uint32_t gainDelay;
        uint32_t exposureDelay;
        uint32_t sensorMetadata;
   };

The generated structs will also have two constructors, a constructor that
fills all fields with the default values, and a second constructor that takes
a value for every field. The default value constructor will fill in the fields
with the specified default value if it exists. In the above example, `gainDelay_`
will be initialized to 1. If no default value is specified, then it will be
filled in as zero (or -1 for a FileDescriptor type).

All fields and constructors/destructors in these generated structs are public.

Using the IPA interface (pipeline handler)
------------------------------------------

The following headers are necessary to use an IPA in the pipeline handler
(with raspberrypi as an example):

.. code-block:: C++

   #include <libcamera/ipa/raspberrypi_ipa_interface.h>
   #include <libcamera/ipa/raspberrypi_ipa_proxy.h>

The first header includes definitions of the custom data structures, and
the definition of the complete IPA interface (including both the Main and
the Event IPA interfaces). The name of the header file comes from the name
of the mojom file, which in this case was raspberrypi.mojom.

The second header includes the definition of the specialized IPA proxy. It
exposes the complete IPA interface. We will see how to use it in this section.

In the pipeline handler, we first need to construct a specialized IPA proxy.
From the point of view of the pipeline hander, this is the object that is the
IPA.

To do so, we invoke the IPAManager:

.. code-block:: C++

        std::unique_ptr<ipa::rpi::IPAProxyRPi> ipa_ =
                IPAManager::createIPA<ipa::rpi::IPAProxyRPi>(pipe_, 1, 1);

The ipa::rpi namespace comes from the namespace that we defined in the mojo
data definition file, in the "Namespacing" section. The name of the proxy,
IPAProxyRPi, comes from the name given to the main IPA interface,
IPARPiInterface, in the "The Main IPA interface" section.

The return value of IPAManager::createIPA shall be error-checked, to confirm
that the returned pointer is not a nullptr.

After this, before initializing the IPA, slots should be connected to all of
the IPA's signals, as defined in the Event IPA interface:

.. code-block:: C++

	ipa_->statsMetadataComplete.connect(this, &RPiCameraData::statsMetadataComplete);
	ipa_->runIsp.connect(this, &RPiCameraData::runIsp);
	ipa_->embeddedComplete.connect(this, &RPiCameraData::embeddedComplete);
	ipa_->setIsp.connect(this, &RPiCameraData::setIsp);
	ipa_->setStaggered.connect(this, &RPiCameraData::setStaggered);

The slot functions have a function signature based on the function definition
in the Event IPA interface. All plain old data (POD) types are as-is (with
their C++ versions, eg. uint32 -> uint32_t), and all structs are const references.

For example, for the following entry in the Event IPA interface:

.. code-block:: none

   statsMetadataComplete(uint32 bufferId, ControlList controls);

A function with the following function signature shall be connected to the
signal:

.. code-block:: C++

   void statsMetadataComplete(uint32_t bufferId, const ControlList &controls);

After connecting the slots to the signals, the IPA should be initialized
(using the main interface definition example from earlier):

.. code-block:: C++

   IPASettings settings{};
   bool metadataSupport;
   int ret = ipa_->init(settings, "sensor name", &metadataSupport);

At this point, any IPA functions that were defined in the Main IPA interface
can be called as if they were regular member functions, for example (based on
the main interface definition example from earlier):

.. code-block:: C++

   ipa_->start();
   int ret = ipa_->configure(sensorInfo_, streamConfig, entityControls, ipaConfig, &result);
   ipa_->signalStatReady(RPi::BufferMask::STATS | static_cast<unsigned int>(index));

Remember that any functions designated as asynchronous *must not* be called
before start().

Notice that for both init() and configure(), the first output parameter is a
direct return, since it is an int32, while the other output parameter is a
pointer-based output parameter.

Using the IPA interface (IPA Module)
------------------------------------

The following header is necessary to implement an IPA Module (with raspberrypi
as an example):

.. code-block:: C++

   #include <libcamera/ipa/raspberrypi_ipa_interface.h>

This header includes definitions of the custom data structures, and
the definition of the complete IPA interface (including both the Main and
the Event IPA interfaces). The name of the header file comes from the name
of the mojom file, which in this case was raspberrypi.mojom.

The IPA module must implement the IPA interface class that is defined in the
header. In the case of our example, that is ipa::rpi::IPARPiInterface. The
ipa::rpi namespace comes from the namespace that we defined in the mojo data
definition file, in the "Namespacing" section. The name of the interface is the
same as the name given to the Main IPA interface.

The function signature rules are the same as for the slots in the pipeline
handler side; PODs are passed by value, and structs are passed by const
reference. For the Main IPA interface, output values are also allowed (only
for synchronous calls), so there may be output parameters as well. If the
first output parameter is a POD it will be returned by value, otherwise
it will be returned by an output parameter pointer. The second and any other
output parameters will also be returned by output parameter pointers.

For example, for the following function specification in the Main IPA interface
definition:

.. code-block:: none

   configure(libcamera.IPACameraSensorInfo sensorInfo,
             uint32 exampleNumber,
             map<uint32, libcamera.IPAStream> streamConfig,
             map<uint32, libcamera.ControlInfoMap> entityControls,
             ConfigInput ipaConfig)
   => (int32 ret, ConfigOutput results);

We will need to implement a function with the following function signature:

.. code-block:: C++

        int configure(const IPACameraSensorInfo &sensorInfo,
                      uint32_t exampleNumber,
                      const std::map<unsigned int, IPAStream> &streamConfig,
                      const std::map<unsigned int, ControlInfoMap> &entityControls,
                      const ipa::rpi::ConfigInput &data,
                      ipa::rpi::ConfigOutput *response);

The return value is int, because the first output parameter is int32.  The rest
of the output parameters (in this case, only response) become output parameter
pointers. The non-POD input parameters become const references, and the POD
input parameter is passed by value.

At any time after start() and before stop() (though usually only in response to
an IPA call), the IPA may send data to the pipeline handler by emitting
signals. These signals are defined in the C++ IPA interface class (which is in
the generated and included header).

For example, for the following function defined in the Event IPA interface:

.. code-block:: none

   statsMetadataComplete(uint32 bufferId, libcamera.ControlList controls);

We can emit a signal like so:

.. code-block:: C++

   statsMetadataComplete.emit(bufferId & RPi::BufferMask::ID, libcameraMetadata_);
