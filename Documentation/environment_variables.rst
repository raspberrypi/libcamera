.. SPDX-License-Identifier: CC-BY-SA-4.0

Environment variables
=====================

The libcamera behaviour can be tuned through environment variables. This
document lists all the available variables and describes their usage.

List of variables
-----------------

LIBCAMERA_LOG_FILE
   The custom destination for log output.

   Example value: ``/home/{user}/camera_log.log``

LIBCAMERA_LOG_LEVELS
   Configure the verbosity of log messages for different categories (`more <Log levels_>`__).

   Example value: ``*:DEBUG``

LIBCAMERA_LOG_NO_COLOR
   Disable coloring of log messages (`more <Notes about debugging_>`__).

LIBCAMERA_IPA_CONFIG_PATH
   Define custom search locations for IPA configurations (`more <IPA configuration_>`__).

   Example value: ``${HOME}/.libcamera/share/ipa:/opt/libcamera/vendor/share/ipa``

LIBCAMERA_IPA_FORCE_ISOLATION
   When set to a non-empty string, force process isolation of all IPA modules.

   Example value: ``1``

LIBCAMERA_IPA_MODULE_PATH
   Define custom search locations for IPA modules (`more <IPA module_>`__).

   Example value: ``${HOME}/.libcamera/lib:/opt/libcamera/vendor/lib``

Further details
---------------

Notes about debugging
~~~~~~~~~~~~~~~~~~~~~

The environment variables ``LIBCAMERA_LOG_FILE``, ``LIBCAMERA_LOG_LEVELS`` and
``LIBCAMERA_LOG_NO_COLOR`` are used to modify the default configuration of the
libcamera logger.

By default, libcamera logs all messages to the standard error (std::cerr).
Messages are colored by default depending on the log level. Coloring can be
disabled by setting the ``LIBCAMERA_LOG_NO_COLOR`` environment variable.

The default log destination can also be directed to a file by setting the
``LIBCAMERA_LOG_FILE`` environment variable to the log file name. This also
disables coloring.

Log levels are controlled through the ``LIBCAMERA_LOG_LEVELS`` variable, which
accepts a comma-separated list of 'category:level' pairs.

The `level <Log levels_>`__ part is mandatory and can either be specified by
name or by numerical index associated with each level.

The optional `category <Log categories_>`__ is a string matching the categories
defined by each file in the source base using the logging infrastructure. It
can include a wildcard ('*') character at the end to match multiple categories.

For more information refer to the `API documentation <https://libcamera.org/api-html/log_8h.html#details>`__.

Examples:

Enable full debug output to a separate file, for every `category <Log categories_>`__
within a local environment:

.. code:: bash

   :~$ LIBCAMERA_LOG_FILE='/tmp/example_log.log' \
       LIBCAMERA_LOG_LEVELS=0 \
       cam --list

Enable full debug output for the categories ``Camera`` and ``V4L2`` within a
global environment:

.. code:: bash

   :~$ export LIBCAMERA_LOG_LEVELS='Camera:DEBUG,V4L2:DEBUG'
   :~$ cam --list

Log levels
~~~~~~~~~~

This is the list of available log levels, notice that all levels below
the chosen one are printed, while those above are discarded.

-  DEBUG (0)
-  INFO (1)
-  WARN (2)
-  ERROR (3)
-  FATAL (4)

Example:
If you choose WARN (2), you will be able to see WARN (2), ERROR (3) and FATAL (4)
but not DEBUG (0) and INFO (1).

Log categories
~~~~~~~~~~~~~~

Every category represents a specific area of the libcamera codebase,
the names can be located within the source code, for example:
`src/libcamera/camera_manager.cpp <https://git.libcamera.org/libcamera/libcamera.git/tree/src/libcamera/camera_manager.cpp#n35>`__

.. code:: cpp

   LOG_DEFINE_CATEGORY(Camera)

There are two available macros used to assign a category name to a part of the
libcamera codebase:

LOG_DEFINE_CATEGORY
   This macro is required, in order to use the ``LOGC`` macro for a particular
   category. It can only be used once for each category. If you want to create
   log messages within multiple compilation units for the same category utilize
   the ``LOG_DECLARE_CATEGORY`` macro, in every file except the definition file.
LOG_DECLARE_CATEGORY
   Used for sharing an already defined category between multiple separate
   compilation units.

Both macros have to be used within the libcamera namespace of the C++ source
code.

IPA configuration
~~~~~~~~~~~~~~~~~

IPA modules use configuration files to store parameters. The format and
contents of the configuration files is specific to the IPA module. They usually
contain tuning parameters for the algorithms, in JSON format.

The ``LIBCAMERA_IPA_CONFIG_PATH`` variable can be used to specify custom
storage locations to search for those configuration files.

`Examples <https://git.libcamera.org/libcamera/libcamera.git/tree/src/ipa/raspberrypi/data>`__

IPA module
~~~~~~~~~~

In order to locate the correct IPA module for your hardware, libcamera gathers
existing IPA modules from multiple locations. The default locations for this
operation are the installed system path (for example on Debian:
``/usr/local/x86_64-pc-linux-gnu/libcamera``) and the build directory.
With the ``LIBCAMERA_IPA_MODULE_PATH``, you can specify a non-default location
to search for IPA modules.
