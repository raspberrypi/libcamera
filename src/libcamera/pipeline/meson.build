# SPDX-License-Identifier: CC0-1.0

if pipelines.contains('raspberrypi/bcm2835') or pipeline.contains('raspberrypi/pisp')
    subdir('raspberrypi/common')
endif

foreach pipeline : pipelines
    subdir(pipeline)
endforeach
