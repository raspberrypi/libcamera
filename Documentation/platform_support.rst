.. SPDX-License-Identifier: CC-BY-SA-4.0

Platform Support
================

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

.. toctree::
   :hidden:

   ISP feature support matrix <isp-feature-matrix>
   Camera Sensor support <sensor-support>
