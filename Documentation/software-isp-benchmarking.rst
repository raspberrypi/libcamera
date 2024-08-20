.. SPDX-License-Identifier: CC-BY-SA-4.0

.. include:: documentation-contents.rst

.. _software-isp-benchmarking:

Software ISP benchmarking
=========================

The Software ISP is particularly sensitive to performance regressions therefore
it is a good idea to always benchmark the Software ISP before and after making
changes to it and ensure that there are no performance regressions.

DebayerCpu class builtin benchmark
----------------------------------

The DebayerCpu class has a builtin benchmark. This benchmark measures the time
spent on processing (collecting statistics and debayering) only, it does not
measure the time spent on capturing or outputting the frames.

The builtin benchmark always runs. So this can be used by simply running "cam"
or "qcam" with a pipeline using the Software ISP.

When it runs it will skip measuring the first 30 frames to allow the caches and
the CPU temperature (turbo-ing) to warm-up and then it measures 30 fps and shows
the total and per frame processing time using an info level log message:

.. code-block:: text

   INFO Debayer debayer_cpu.cpp:907 Processed 30 frames in 244317us, 8143 us/frame

To get stable measurements it is advised to disable any other processes which
may cause significant CPU usage (e.g. disable wifi, bluetooth and browsers).
When possible it is also advisable to disable CPU turbo-ing and
frequency-scaling.

For example when benchmarking on a Lenovo ThinkPad X1 Yoga Gen 8, with the
charger plugged in, the CPU can be fixed to run at 2 GHz using:

.. code-block:: shell

   sudo x86_energy_perf_policy --turbo-enable 0
   sudo cpupower frequency-set -d 2GHz -u 2GHz

with these settings the builtin bench reports a processing time of ~7.8ms/frame
on this laptop for FHD SGRBG10 (unpacked) bayer data.

Measuring power consumption
---------------------------

Since the Software ISP is often used on mobile devices it is also important to
measure power consumption and ensure that that does not regress.

For example to measure power consumption on a Lenovo ThinkPad X1 Yoga Gen 8 it
needs to be running on battery and it should be configured with its
platform-profile (/sys/firmware/acpi/platform_profile) set to balanced and with
its default turbo and frequency-scaling behavior to match real world usage.

Then start qcam to capture a FHD picture at 30 fps and position the qcam window
so that it is fully visible. After this run the following command to monitor the
power consumption:

.. code-block:: shell

   watch -n 10 cat /sys/class/power_supply/BAT0/power_now /sys/class/hwmon/hwmon6/fan?_input

Note this not only measures the power consumption in µW it also monitors the
speed of this laptop's 2 fans. This is important because depending on the
ambient temperature the 2 fans may spin up while testing and this will cause an
additional power consumption of approx. 0.5 W messing up the measurement.

After starting qcam + the watch command let the laptop sit without using it for
2 minutes for the readings to stabilize. Then check that the fans have not
turned on and manually take a couple of consecutive power readings and average
these.

On the example Lenovo ThinkPad X1 Yoga Gen 8 laptop this results in a measured
power consumption of approx. 13 W while running qcam versus approx. 4-5 W while
setting idle with its OLED panel on.
