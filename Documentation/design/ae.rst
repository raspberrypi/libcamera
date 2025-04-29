.. SPDX-License-Identifier: CC-BY-SA-4.0

Design of Exposure and Gain controls
====================================

This document explains the design and rationale of the controls related to
exposure and gain. This includes the all-encompassing auto-exposure (AE), the
manual exposure control, and the manual gain control.

Description of the problem
--------------------------

Sub controls
^^^^^^^^^^^^

There are more than one control that make up total exposure: exposure time,
gain, and aperture (though for now we will not consider aperture). We already
had individual controls for setting the values of manual exposure and manual
gain, but for switching between auto mode and manual mode we only had a
high-level boolean AeEnable control that would set *both* exposure and gain to
auto mode or manual mode; we had no way to set one to auto and the other to
manual.

So, we need to introduce two new controls to act as "levers" to indicate
individually for exposure and gain if the value would come from AEGC or if it
would come from the manual control value.

Aperture priority
^^^^^^^^^^^^^^^^^

We eventually may need to support aperture, and so whatever our solution is for
having only some controls on auto and the others on manual needs to be
extensible.

Flickering when going from auto to manual
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

When a manual exposure or gain value is requested by the application, it costs
a few frames worth of time for them to take effect. This means that during a
transition from auto to manual, there would be flickering in the control values
and the transition won't be smooth.

Take for instance the following flow, where we start on auto exposure (which
for the purposes of the example increments by 1 each frame) and we want to
switch seamlessly to manual exposure, which involves copying the exposure value
computed by the auto exposure algorithm:

::

                +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
                | N   | | N+1 | | N+2 | | N+3 | | N+4 | | N+5 | | N+6 |
                +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+

 Mode requested: Auto    Auto    Auto    Manual  Manual  Manual  Manual
 Exp requested:  N/A     N/A     N/A     2       2       2       2
 Set in Frame:   N+2     N+3     N+4     N+5     N+6     N+7     N+8

 Mode used:      Auto    Auto    Auto    Auto    Auto    Manual  Manual
 Exp used:       0       1       2       3       4       2       2

As we can see, after frame N+2 completes, we copy the exposure value that was
used for frame N+2 (which was computed by AE algorithm), and queue that value
into request N+3 with manual mode on. However, as it takes two frames for the
exposure to be set, the exposure still changes since it is set by AE, and we
get a flicker in the exposure during the switch from auto to manual.

A solution is to *not submit* any exposure value when manual mode is enabled,
and wait until the manual mode as been "applied" before copying the exposure
value:

::

                +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
                | N   | | N+1 | | N+2 | | N+3 | | N+4 | | N+5 | | N+6 |
                +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+

 Mode requested: Auto    Auto    Auto    Manual  Manual  Manual  Manual
 Exp requested:  N/A     N/A     N/A     None    None    None    5
 Set in Frame:   N+2     N+3     N+4     N+5     N+6     N+7     N+8

 Mode used:      Auto    Auto    Auto    Auto    Auto    Manual  Manual
 Exp used:       0       1       2       3       4       5       5

In practice, this works. However, libcamera has a policy where once a control
is submitted, its value is saved and does not need to be resubmitted. If the
manual exposure value was set while auto mode was on, in theory the value would
be saved, so when manual mode is enabled, the exposure value that was
previously set would immediately be used. Clearly this solution isn't correct,
but it can serve as the basis for a proper solution, with some more rigorous
rules.

Existing solutions
------------------

Raspberry Pi
^^^^^^^^^^^^

The Raspberry Pi IPA gets around the lack of individual AeEnable controls for
exposure and gain by using magic values. When AeEnable is false, if one of the
manual control values was set to 0 then the value computed by AEGC would be
used for just that control. This solution isn't desirable, as it prevents
that magic value from being used as a valid value.

To get around the flickering issue, when AeEnable is false, the Raspberry Pi
AEGC simply stops updating the values to be set, without restoring the
previously set manual exposure time and gain. This works, but is not a proper
solution.

Android
^^^^^^^

The Android HAL specification requires that exposure and gain (sensitivity)
must both be manual or both be auto. It cannot be that one is manual while the
other is auto, so they simply don't support sub controls.

For the flickering issue, the Android HAL has an AeLock control. To transition
from auto to manual, the application would keep AE on auto, and turn on the
lock. Once the lock has propagated through, then the value can be copied from
the result into the request and the lock disabled and the mode set to manual.

The problem with this solution is, besides the extra complexity, that it is
ambiguous what happens if there is a state transition from manual to locked
(even though it's a state transition that doesn't make sense). If locked is
defined to "use the last automatically computed values" then it could use the
values from the last time it AE was set to auto, or it would be undefined if AE
was never auto (eg. it started out as manual), or if AE is implemented to run
in the background it could just use the current values that are computed. If
locked is defined to "use the last value that was set" there would be less
ambiguity. Still, it's better if we can make it impossible to execute this
nonsensical state transition, and if we can reduce the complexity of having
this extra control or extra setting on a lever.

Summary of goals
----------------

- We need a lock of some sort, to instruct the AEGC to not update output
  results

- We need manual modes, to override the values computed by the AEGC

- We need to support seamless transitions from auto to manual, and do so
  without flickering

- We need custom minimum values for the manual controls; that is, no magic
  values for enabling/disabling auto

- All of these need to be done with AE sub-controls (exposure time, analogue
  gain) and be extensible to aperture in the future

Our solution
------------

A diagram of our solution:

::

  +----------------------------+-------------+------------------+-----------------+
  |          INPUT             |  ALGORITHM  |     RESULT       |     OUTPUT      |
  +----------------------------+-------------+------------------+-----------------+

    ExposureTimeMode                                             ExposureTimeMode
  ---------------------+----------------------------------------+----------------->
    0: Auto            |                                        |
    1: Manual          |                                        V
                       |                                       |\
                       |                                       | \
                       |  /----------------------------------> | 1|  ExposureTime
                       |  |    +-------------+  exposure time  |  | -------------->
                       \--)--> |             | --------------> | 0|
    ExposureTime          |    |             |                 | /
  ------------------------+--> |             |                 |/
                               |             |                       AeState
                               |     AEGC    | ----------------------------------->
    AnalogueGain               |             |
  ------------------------+--> |             |                 |\
                          |    |             |                 | \
                       /--)--> |             | --------------> | 0|  AnalogueGain
                       |  |    +-------------+  analogue gain  |  | -------------->
                       |  \----------------------------------> | 1|
                       |                                       | /
                       |                                       |/
                       |                                        ^
    AnalogueGainMode   |                                        | AnalogueGainMode
  ---------------------+----------------------------------------+----------------->
    0: Auto
    1: Manual

  AeEnable
    - True -> ExposureTimeMode:Auto + AnalogueGainMode:Auto
    - False -> ExposureTimeMode:Manual + AnalogueGainMode:Manual


The diagram is divided in four sections horizontally:

- Input: The values received from the request controls

- Algorithm: The algorithm itself

- Result: The values calculated by the algorithm

- Output: The values reported in result metadata and applied to the device

The four input controls are divided between manual values (ExposureTime and
AnalogueGain), and operation modes (ExposureTimeMode and AnalogueGainMode). The
former are the manual values, the latter control how they're applied. The two
modes are independent from each other, and each can take one of two values:

- Auto (0): The AGC computes the value normally. The AGC result is applied
  to the output. The manual value is ignored *and is not retained*.

- Manual (1): The AGC uses the manual value internally. The corresponding
  manual control from the request is applied to the output. The AGC result
  is ignored.

The AeState control reports the state of the unified AEGC block. If both
ExposureTimeMode and AnalogueGainMode are set to manual then it will report
Idle. If at least one of the two is set to auto, then AeState will report
if the AEGC has Converged or not (Searching). This control replaces the old
AeLocked control, as it was insufficient for reporting the AE state.

There is a caveat to manual mode: the manual control value is not retained if
it is set during auto mode. This means that if manual mode is entered without
also setting the manual value, then it will enter a state similar to "locked",
where the last automatically computed value while the mode was auto will be
used. Once the manual value is set, then that will be used and retained as
usual.

This simulates an auto -> locked -> manual or auto -> manual state transition,
and makes it impossible to do the nonsensical manual -> locked state
transition.

AeEnable still exists to allow applications to set the mode of all the
sub-controls at once. Besides being for convenience, this will also be useful
when we eventually implement an aperture control. This is because applications
that will be made before aperture will have been available would still be able
to set aperture mode to auto or manual, as opposed to having the aperture stuck
at auto while the application really wanted manual. Although the aperture would
still be stuck at an uncontrollable value, at least it would be at a static
usable value as opposed to varying via the AEGC algorithm.

With this solution, the earlier example would become:

::

                 +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
                 | N+2 | | N+3 | | N+4 | | N+5 | | N+6 | | N+7 | | N+8 | | N+9 | | N+10|
                 +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+ +-----+
 Mode requested:  Auto    Manual  Manual  Manual  Manual  Manual  Manual  Manual  Manual
 Exp requested:   N/A     None    None    None    None    10      None    10      10
 Set in Frame:    N+4     N+5     N+6     N+7     N+8     N+9     N+10    N+11    N+12

 Mode used:       Auto    Auto    Auto    Manual  Manual  Manual  Manual  Manual  Manual
 Exp used:        2       3       4       5       5       5       5       10      10

This example is extended by a few frames to exhibit the simulated "locked"
state. At frame N+5 the application has confirmed that the manual mode has been
entered, but does not provide a manual value until request N+7. Thus, the value
that is used in requests N+5 and N+6 (where the mode is disabled), comes from
the last value that was used when the mode was auto, which comes from frame
N+4.

Then, in N+7, a manual value of 10 is supplied. It takes until frame N+9 for
the exposure to be applied. N+8 does not supply a manual value, but the last
supplied value is retained, so a manual value of 10 is still used and set in
frame N+10.

Although this behavior is the same as what we had with waiting for the manual
mode to propagate (in the section "Description of the problem"), this time it
is correct as we have defined specifically that if a manual value was specified
while the mode was auto, it will not be retained.

Description of the controls
---------------------------

As described above, libcamera offers the following controls related to exposure
and gain:

- AnalogueGain

- AnalogueGainMode

- ExposureTime

- ExposureTimeMode

- AeState

- AeEnable

Auto-exposure and auto-gain can be enabled and disabled separately using the
ExposureTimeMode and AnalogueGainMode controls respectively. The AeEnable
control can also be used, as it sets both of the modes simultaneously. The
AeEnable control is not returned in metadata.

When the respective mode is set to auto, the respective value that is computed
by the AEGC algorithm is applied to the image sensor. Any value that is
supplied in the manual ExposureTime/AnalogueGain control is ignored and not
retained. Another way to understand this is that when the mode transitions from
auto to manual, the internally stored control value is overwritten with the
last value computed by the auto algorithm.

This means that when we transition from auto to manual without supplying a
manual control value, the last value that was set by the AEGC algorithm will
keep be used. This can be used to do a flickerless transition from auto to
manual as described earlier. If the camera started out in manual mode and no
corresponding value has been supplied yet, then a best-effort default value
shall be set.

The manual control value can be set in the same request as setting the mode to
auto if the desired manual control value is already known.

Transitioning from manual to auto shall be implicitly flickerless, as the AEGC
algorithms are expected to start running from the last manual value.

The AeState metadata reports the state of the AE algorithm. As AE cannot
compute exposure and gain separately, the state of the AE component is
unified. There are three states: Idle, Searching, and Converged.

The state shall be Idle if both ExposureTimeMode and AnalogueGainMode
are set to Manual. If the camera only supports one of the two controls,
then the state shall be Idle if that one control is set to Manual. If
the camera does not support Manual for at least one of the two controls,
then the state will never be Idle, as AE will always be running.

The state shall be Searching if at least one of exposure or gain calculated
by the AE algorithm is used (that is, at least one of the two modes is Auto),
*and* the value(s) have not converged yet.

The state shall be Converged if at least one of exposure or gain calculated
by the AE algorithm is used (that is, at least one of the two modes is Auto),
*and* the value(s) have converged.
