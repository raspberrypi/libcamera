.. SPDX-License-Identifier: CC-BY-SA-4.0

Camera Sensor Support
=====================

Support for camera sensors in libcamera continues to grow and can be easily
extended to any sensor supported by the Linux kernel, as well as to custom
devices or configurations.

While some integration effort may be necessary, any supported camera can be
used on any supported platform, as long as the required physical hardware
connections are properly handled.

Existing support includes the following camera sensors:

.. list-table::
   :header-rows: 1

   * - Vendor
     - Models
   * - Sony
     - IMX214, IMX219, IMX258, IMX283, IMX290, IMX296, IMX327, IMX335, IMX415,
       IMX462, IMX477, IMX519, IMX708
   * - Omnivision
     - OV2685, OV2740, OV4689, OV5640, OV5647, OV5670, OV5675, OV5693, OV7251,
       OV8858, OV8865, OV9281, OV13858, OV64A40
   * - On-Semi
     - AR0144, AR0521
   * - ST-Microelectronics
     - VD56G3
   * - Galaxy Core
     - GC05A2, GC08A3
