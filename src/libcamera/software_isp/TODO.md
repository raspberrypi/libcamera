<!-- SPDX-License-Identifier: CC0-1.0 -->

# Software ISP TODO list

This file contains the TODO list for the software ISP.

## Common code and CPU-based implementation

The TODO items in this section gather comments from patch review that were not
deemed to require being addressed right away. The text in block quotes is
copied directly from e-mail review.

### Reconsider stats sharing

```
>>> +void SwStatsCpu::finishFrame(void)
>>> +{
>>> +	*sharedStats_ = stats_;
>> 
>> Is it more efficient to copy the stats instead of operating directly on
>> the shared memory ?
>
> I inherited doing things this way from Andrey. I kept this because
> we don't really have any synchronization with the IPA reading this.
>
> So the idea is to only touch this when the next set of statistics
> is ready since we don't know when the IPA is done with accessing
> the previous set of statistics ...
>
> This is both something which seems mostly a theoretic problem,
> yet also definitely something which I think we need to fix.
>
> Maybe use a ringbuffer of stats buffers and pass the index into
> the ringbuffer to the emit signal ?

That would match how we deal with hardware ISPs, and I think that's a
good idea. It will help decoupling the processing side from the IPA.
```

### Remove statsReady signal

```
> class SwStatsCpu
> {
> 	/**
> 	 * \brief Signals that the statistics are ready
> 	 */
> 	Signal<> statsReady;

But better, I wonder if the signal could be dropped completely. The
SwStatsCpu class does not operate asynchronously. Shouldn't whoever
calls the finishFrame() function then handle emitting the signal ?

Now, the trouble is that this would be the DebayerCpu class, whose name
doesn't indicate as a prime candidate to handle stats. However, it
already exposes a getStatsFD() function, so we're already calling for
trouble :-) Either that should be moved to somewhere else, or the class
should be renamed. Considering that the class applies colour gains in
addition to performing the interpolation, it may be more of a naming
issue.

Removing the signal and refactoring those classes doesn't have to be
addressed now, I think it would be part of a larger refactoring
(possibly also considering platforms that have no ISP but can produce
stats in hardware, such as the i.MX7), but please keep it on your radar.
```

### Store ISP parameters in per-frame buffers

```
> /**
>  * \fn void Debayer::process(FrameBuffer *input, FrameBuffer *output, DebayerParams params)
>  * \brief Process the bayer data into the requested format.
>  * \param[in] input The input buffer.
>  * \param[in] output The output buffer.
>  * \param[in] params The parameters to be used in debayering.
>  *
>  * \note DebayerParams is passed by value deliberately so that a copy is passed
>  * when this is run in another thread by invokeMethod().
>  */

Possibly something to address later, by storing ISP parameters in
per-frame buffers like we do for hardware ISPs.
```

### DebayerCpu cleanups

```
> >> class DebayerCpu : public Debayer, public Object
> >>   const SharedFD &getStatsFD() { return stats_->getStatsFD(); }
> >
> > This,
>
> Note the statistics pass-through stuff is sort of a necessary evil
> since we want one main loop going over the data line by line and
> doing both debayering as well as stats while the line is still
> hot in the l2 cache. And things like the process2() and process4()
> loops are highly CPU debayering specific so I don't think we should
> move those out of the CpuDebayer code.

Yes, that I understood from the review. "necessary evil" is indeed the
right term :-) I expect it will take quite some design skills to balance
the need for performances and the need for a maintainable architecture.

> > plus the fact that this class handles colour gains and gamma,
> > makes me thing we have either a naming issue, or an architecture issue.
>
> I agree that this does a bit more then debayering, although
> the debayering really is the main thing it does.
>
> I guess the calculation of the rgb lookup tables which do the
> color gains and gamma could be moved outside of this class,
> that might even be beneficial for GPU based debayering assuming
> that that is going to use rgb lookup tables too (it could
> implement actual color gains + gamma correction in some different
> way).
>
> I think this falls under the lets wait until we have a GPU
> based SoftISP MVP/POC and then do some refactoring to see which
> bits should go where.
```

### Decouple pipeline and IPA naming

```
> The current src/ipa/meson.build assumes the IPA name to match the
> pipeline name. For this reason "-Dipas=simple" is used for the
> Soft IPA module.

This should be addressed.
```

### Doxyfile cleanup

```
>> diff --git a/Documentation/Doxyfile.in b/Documentation/Doxyfile.in
>> index a86ea6c1..2be8d47b 100644
>> --- a/Documentation/Doxyfile.in
>> +++ b/Documentation/Doxyfile.in
>> @@ -44,6 +44,7 @@ EXCLUDE                = @TOP_SRCDIR@/include/libcamera/base/span.h \
>>                            @TOP_SRCDIR@/src/libcamera/pipeline/ \
>>                            @TOP_SRCDIR@/src/libcamera/tracepoints.cpp \
>>                            @TOP_BUILDDIR@/include/libcamera/internal/tracepoints.h \
>> +                         @TOP_BUILDDIR@/include/libcamera/ipa/soft_ipa_interface.h \
> Why is this needed ?
>
>>                            @TOP_BUILDDIR@/src/libcamera/proxy/
>>     EXCLUDE_PATTERNS       = @TOP_BUILDDIR@/include/libcamera/ipa/*_serializer.h \
>> diff --git a/include/libcamera/ipa/meson.build b/include/libcamera/ipa/meson.build
>> index f3b4881c..3352d08f 100644
>> --- a/include/libcamera/ipa/meson.build
>> +++ b/include/libcamera/ipa/meson.build
>> @@ -65,6 +65,7 @@ pipeline_ipa_mojom_mapping = {
>>       'ipu3': 'ipu3.mojom',
>>       'rkisp1': 'rkisp1.mojom',
>>       'rpi/vc4': 'raspberrypi.mojom',
>> +    'simple': 'soft.mojom',
>>       'vimc': 'vimc.mojom',
>>   }
>>   diff --git a/include/libcamera/ipa/soft.mojom b/include/libcamera/ipa/soft.mojom
>> new file mode 100644
>> index 00000000..c249bd75
>> --- /dev/null
>> +++ b/include/libcamera/ipa/soft.mojom
>> @@ -0,0 +1,28 @@
>> +/* SPDX-License-Identifier: LGPL-2.1-or-later */
>> +
>> +/*
>> + * \todo Document the interface and remove the related EXCLUDE_PATTERNS entry.
> Ah that's why.

Yes, because, well... all the other IPAs were doing that...

> It doesn't have to be done before merging, but could you
> address this sooner than later ?
```

### Improve black level and colour gains application

```
I think the black level should eventually be moved before debayering, and
ideally the colour gains as well. I understand the need for optimizations to
lower the CPU consumption, but at the same time I don't feel comfortable
building up on top of an implementation that may work a bit more by chance than
by correctness, as that's not very maintainable.
```

## GPU-based implementation

The TODO items are listed in perceived order of ease.

### Denoising

WIP

### Dead pixel correction

WIP

### Lense shading correction

WIP

### Use dma-buf handle to generate upload texture

`eglCreateImageKHR` can be used to generate the upload texture i.e.to feed the
bayer data into the GPU.

### processFrame() to run in its own thread

`processFrame()` runs in the context of the Debayer::process() thread. Robert
Mader suggested and it seems like a good suggestion too to run processFrame()
in its own thread.

### Multi-pass shaders

- This needs some rewiring the idea is to have a list of algorithms as is done
  in cpuisp iterating through the list in a for() loop.
- The logic managing the loop has an initial input buffer and the final output
  buffer.
- The higher level logic must then inform each of the algorithms either to
  generate an internal working buffer or pass the final output buffer to the
  last shader in the list
- This will allow for multi-pass shaders with the final algorithm presenting
  data not to its internal buffer but to the final output buffer

### 24 bit output support

Need to implement compute shader to do this.

### Lense flare correction

Not WIP still TBD
