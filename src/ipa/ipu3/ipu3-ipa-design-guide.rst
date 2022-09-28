.. SPDX-License-Identifier: CC-BY-SA-4.0

IPU3 IPA Architecture Design and Overview
=========================================

The IPU3 IPA is built as a modular and extensible framework with an
upper layer to manage the interactions with the pipeline handler, and
the image processing algorithms split to compartmentalise the processing
required for each processing block, making use of the fixed-function
accelerators provided by the ImgU ISP.

The core IPU3 class is responsible for initialisation and construction
of the algorithm components, processing controls set by the requests
from applications, and managing events from the pipeline handler.

::

      ┌───────────────────────────────────────────┐
      │      IPU3 Pipeline Handler                │
      │   ┌────────┐    ┌────────┐    ┌────────┐  │
      │   │        │    │        │    │        │  │
      │   │ Sensor ├───►│  CIO2  ├───►│  ImgU  ├──►
      │   │        │    │        │    │        │  │
      │   └────────┘    └────────┘    └─▲────┬─┘  │    P: Parameter Buffer
      │                                 │P   │    │    S: Statistics Buffer
      │                                 │    │S   │
      └─┬───┬───┬──────┬────┬────┬────┬─┴────▼─┬──┘    1: init()
        │   │   │      │ ▲  │ ▲  │ ▲  │ ▲      │       2: configure()
        │1  │2  │3     │4│  │4│  │4│  │4│      │5      3: mapBuffers(), start()
        │   │   │      │ │  │ │  │ │  │ │      │       4: (▼) queueRequest(), fillParamsBuffer(), processStatsBuffer()
        ▼   ▼   ▼      ▼ │  ▼ │  ▼ │  ▼ │      ▼          (▲) setSensorControls, paramsBufferReady, metadataReady Signals
      ┌──────────────────┴────┴────┴────┴─────────┐    5: stop(), unmapBuffers()
      │ IPU3 IPA                                  │
      │                 ┌───────────────────────┐ │
      │ ┌───────────┐   │ Algorithms            │ │
      │ │IPAContext │   │          ┌─────────┐  │ │
      │ │ ┌───────┐ │   │          │ ...     │  │ │
      │ │ │       │ │   │        ┌─┴───────┐ │  │ │
      │ │ │  SC   │ │   │        │ Tonemap ├─┘  │ │
      │ │ │       │ ◄───►      ┌─┴───────┐ │    │ │
      │ │ ├───────┤ │   │      │ AWB     ├─┘    │ │
      │ │ │       │ │   │    ┌─┴───────┐ │      │ │
      │ │ │  FC   │ │   │    │ AGC     ├─┘      │ │
      │ │ │       │ │   │    │         │        │ │
      │ │ └───────┘ │   │    └─────────┘        │ │
      │ └───────────┘   └───────────────────────┘ │
      └───────────────────────────────────────────┘
        SC: IPASessionConfiguration
        FC: IPAFrameContext(s)

The IPA instance is constructed and initialised at the point a Camera is
created by the IPU3 pipeline handler. The initialisation call provides
details about which camera sensor is being used, and the controls that
it has available, along with their default values and ranges.

Buffers
~~~~~~~

The IPA will have Parameter and Statistics buffers shared with it from
the IPU3 Pipeline handler. These buffers will be passed to the IPA using
the ``mapBuffers()`` call before the ``start()`` operation occurs.

The IPA will map the buffers into CPU-accessible memory, associated with
a buffer ID, and further events for sending or receiving parameter and
statistics buffers will reference the ID to avoid expensive memory
mapping operations, or the passing of file handles during streaming.

After the ``stop()`` operation occurs, these buffers will be unmapped
when requested by the pipeline handler using the ``unmapBuffers()`` call
and no further access to the buffers is permitted.

Context
~~~~~~~

Algorithm calls will always have the ``IPAContext`` available to them.
This context comprises of two parts:

-  IPA Session Configuration
-  IPA Frame Context

The session configuration structure ``IPASessionConfiguration``
represents constant parameters determined before streaming commenced
during ``configure()``.

The IPA Frame Context provides the storage for algorithms for a single
frame operation.

The ``IPAFrameContext`` structure may be extended to an array, list, or
queue to store historical state for each frame, allowing algorithms to
obtain and reference results of calculations which are deeply pipelined.
This may only be done if an algorithm needs to know the context that was
applied at the frame the statistics were produced for, rather than the
previous or current frame.

Presently there is a single ``IPAFrameContext`` without historical data,
and the context is maintained and updated through successive processing
operations.

Operating
~~~~~~~~~

There are three main interactions with the algorithms for the IPU3 IPA
to operate when running:

-  configure()
-  queueRequest()
-  fillParamsBuffer()
-  processStatsBuffer()

The configuration phase allows the pipeline-handler to inform the IPA of
the current stream configurations, which is then passed into each
algorithm to provide an opportunity to identify and track state of the
hardware, such as image size or ImgU pipeline configurations.

Pre-frame preparation
~~~~~~~~~~~~~~~~~~~~~

When configured, the IPA is notified by the pipeline handler of the
Camera ``start()`` event, after which incoming requests will be queued
for processing, requiring a parameter buffer (``ipu3_uapi_params``) to
be populated for the ImgU. This is given to the IPA through
``fillParamsBuffer()``, and then passed directly to each algorithm
through the ``prepare()`` call allowing the ISP configuration to be
updated for the needs of each component that the algorithm is
responsible for.

The algorithm should set the use flag (``ipu3_uapi_flags``) for any
structure that it modifies, and it should take care to ensure that any
structure set by a use flag is fully initialised to suitable values.

The parameter buffer is returned to the pipeline handler through the
``paramsBufferReady`` signal, and from there queued to the ImgU along
with a raw frame captured with the CIO2.

Post-frame completion
~~~~~~~~~~~~~~~~~~~~~

When the capture of an image is completed, and successfully processed
through the ImgU, the generated statistics buffer
(``ipu3_uapi_stats_3a``) is given to the IPA through
``processStatsBuffer()``. This provides the IPA with an opportunity to
examine the results of the ISP and run the calculations required by each
algorithm on the new data. The algorithms may require context from the
operations of other algorithms, for example, the AWB might choose to use
a scene brightness determined by the AGC. It is important that the
algorithms are ordered to ensure that required results are determined
before they are needed.

The ordering of the algorithm processing is determined by their
placement in the ``IPU3::algorithms_`` ordered list.

Finally, the IPA metadata for the completed frame is returned back via
the ``metadataReady`` signal.

Sensor Controls
~~~~~~~~~~~~~~~

The AutoExposure and AutoGain (AGC) algorithm differs slightly from the
others as it requires operating directly on the sensor, as opposed to
through the ImgU ISP. To support this, there is a ``setSensorControls``
signal to allow the IPA to request controls to be set on the camera
sensor through the pipeline handler.
