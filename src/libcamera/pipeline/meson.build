# SPDX-License-Identifier: CC0-1.0

# Location of pipeline specific configuration files
pipeline_data_dir = libcamera_datadir / 'pipeline'

# Allow multi-level directory structuring for the pipeline handlers if needed.
subdirs = []

foreach pipeline : pipelines
    pipeline = pipeline.split('/')[0]
    if pipeline in subdirs
        continue
    endif

    subdir(pipeline)
    subdirs += pipeline
endforeach
