Code to tile up an image for processing by the PiSP.

Essentially we have a wrapper class, Pipeline (pipeline.h), which contains some configuration
and points to all the processing stages. Then we have the abstract Stage class (stages.h) which
is specialised to describe how a tile going through it will change. Most stages have a single
upstream and a single downstream stage, and these can be conveniently derived from BasicStage
(also stages.h), though there are other kinds of stages too (e.g. SplitStage in split_stage.hpp).

The algorithm requires 3 principle methods to be defined on each derived version of the stage
class. Note that the tiling algorithm is agnostic about the direction (X or Y) in which it is
tiling. It merely uses the direction to read the appropriate configuration values for that
direction, but is otherwise the same. The three methods are:

1. void PushStartUp(int output_start, Dir dir)

This method is given the coordinate (in the output image produced as a whole by this stage) of
the place where the output tile must begin. The code must calculate what input coordinate (in the
input image as a whole which the Stage will see) will be required. This is then passed to the
the upstream stage's PushStartUp method. (Normally a stage's input and output images are the
same size, though some stages, such as crops and resizes, will change it.)

2. int PushEndDown(int input_end, Dir dir)

This time we are given the final input pixel coordinate of the tile. We must calculate the final
output pixel coordinate that we can produce given this end point. This value is passed to the
downstream stage's PushEndDown method. (The return value will be described after the following
and final of these three methods.)

3. void PushEndUp(int output_end, Dir dir)

This method takes the final pixel that the downstream stage could actually make (which may be
less that we told it when we invoked PushEndDown on it), and we must calculate the last input
pixel that we require to make it.

PushEndDown and PushEndUp work together. Before PushEndDown returns it invokes PushEndUp and
it is that last input pixel coordinate calculated by PushEndUp that PushEndDown returns. In this
way, calling PushEndDown with a value can can be thought of as asking "here is the maximum
number of pixels that you can have, tell me how many you can actually use".

The progress of the tiling algorithm should be plainly visible in Pipeline::TileDirection. We
work from left to right (or top to bottom). At each point we know where we are in the output
image(s) - given by the end coordinate of the previous tile, or zero for the first tile. This
gets pushed all the way to the top of the pipeline using PushStartUp, starting from each of
our final output stages. Then we can read from the input stages which pixels they will have
to feed downstream. We add the maximum allowed tile size to this, and then push this value all
the way back down using PushEndDown. Recall that PushEndDown implicitly calls PushEndUp, and
this trims back the right hand edge of the tiles so that we don't read pixels that we can't
use.