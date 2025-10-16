.. SPDX-License-Identifier: CC-BY-SA-4.0

Tracing Guide
=============

Guide to tracing in libcamera.

Profiling vs Tracing
--------------------

Tracing is recording timestamps at specific locations. libcamera provides a
tracing facility. This guide shows how to use this tracing facility.

Tracing should not be confused with profiling, which samples execution
at periodic points in time. This can be done with other tools such as
callgrind, perf, gprof, etc., without modification to the application,
and is out of scope for this guide.

Compiling
---------

To compile libcamera with tracing support, it must be enabled through the
meson ``tracing`` option. It depends on the lttng-ust library (available in the
``liblttng-ust-dev`` package for Debian-based distributions).
By default the tracing option in meson is set to ``auto``, so if
liblttng is detected, it will be enabled by default. Conversely, if the option
is set to disabled, then libcamera will be compiled without tracing support.

Defining tracepoints
--------------------

libcamera already contains a set of tracepoints. To define additional
tracepoints, create a file
``include/libcamera/internal/tracepoints/{file}.tp``, where ``file`` is a
reasonable name related to the category of tracepoints that you wish to
define. For example, the tracepoints file for the Request object is called
``request.tp``. An entry for this file must be added in
``include/libcamera/internal/tracepoints/meson.build``.

In this tracepoints file, define your tracepoints `as mandated by lttng
<https://lttng.org/man/3/lttng-ust>`_. The header boilerplate must *not* be
included (as it will conflict with the rest of our infrastructure), and
only the tracepoint definitions (with the ``TRACEPOINT_*`` macros) should be
included.

All tracepoint providers shall be ``libcamera``. According to lttng, the
tracepoint provider should be per-project; this is the rationale for this
decision. To group tracepoint events, we recommend using
``{class_name}_{tracepoint_name}``, for example, ``request_construct`` for a
tracepoint for the constructor of the Request class.

Tracepoint arguments may take C++ objects pointers, in which case the usual
C++ namespacing rules apply. The header that contains the necessary class
definitions must be included at the top of the tracepoint provider file.

Note: the final parameter in ``TP_ARGS`` *must not* have a trailing comma, and
the parameters to ``TP_FIELDS`` are *space-separated*. Not following these will
cause compilation errors.

Using tracepoints (in libcamera)
--------------------------------

To use tracepoints in libcamera, first the header needs to be included:

``#include "libcamera/internal/tracepoints.h"``

Then to use the tracepoint:

``LIBCAMERA_TRACEPOINT({tracepoint_event}, args...)``

This macro must be used, as opposed to lttng's macros directly, because
lttng is an optional dependency of libcamera, so the code must compile and run
even when lttng is not present or when tracing is disabled.

The tracepoint provider name, as declared in the tracepoint definition, is not
included in the parameters of the tracepoint.

There are also two special tracepoints available for tracing IPA calls:

``LIBCAMERA_TRACEPOINT_IPA_BEGIN({pipeline_name}, {ipa_function})``

``LIBCAMERA_TRACEPOINT_IPA_END({pipeline_name}, {ipa_function})``

These shall be placed where an IPA function is called from the pipeline handler,
and when the pipeline handler receives the corresponding response from the IPA,
respectively. These are the tracepoints that our sample analysis script
(see "Analyzing a trace") scans for when computing statistics on IPA call time.

Using tracepoints (from an application)
---------------------------------------

As applications are not part of libcamera, but rather users of libcamera,
applications should seek their own tracing mechanisms. For ease of tracing
the application alongside tracing libcamera, it is recommended to also
`use lttng <https://lttng.org/docs/#doc-tracing-your-own-user-application>`_.

Using tracepoints (from closed-source IPA)
------------------------------------------

Similar to applications, closed-source IPAs can simply use lttng on their own,
or any other tracing mechanism if desired.

Collecting a trace
------------------

A trace can be collected fairly simply from lttng:

.. code-block:: bash

   lttng create $SESSION_NAME
   lttng enable-event -u libcamera:\*
   lttng start
   # run libcamera application
   lttng stop
   lttng view
   lttng destroy $SESSION_NAME

See the `lttng documentation <https://lttng.org/docs/>`_ for further details.

The location of the trace file is printed when running
``lttng create $SESSION_NAME``. After destroying the session, it can still be
viewed by: ``lttng view -t $PATH_TO_TRACE``, where ``$PATH_TO_TRACE`` is the
path that was printed when the session was created. This is the same path that
is used when analyzing traces programatically, as described in the next section.

Analyzing a trace
-----------------

As mentioned above, while an lttng tracing session exists and the trace is not
running, the trace output can be viewed as text by ``lttng view``.

The trace log can also be viewed as text using babeltrace2.  See the
`lttng trace analysis documentation
<https://lttng.org/docs/#doc-viewing-and-analyzing-your-traces-bt>`_
for further details.

babeltrace2 also has a C API and python bindings that can be used to process
traces. See the
`lttng python bindings documentation <https://babeltrace.org/docs/v2.0/python/bt2/>`_
and the
`lttng C API documentation <https://babeltrace.org/docs/v2.0/libbabeltrace2/>`_
for more details.

As an example, there is a script ``utils/tracepoints/analyze-ipa-trace.py``
that gathers statistics for the time taken for an IPA function call, by
measuring the time difference between pairs of events
``libcamera:ipa_call_start`` and ``libcamera:ipa_call_finish``.
