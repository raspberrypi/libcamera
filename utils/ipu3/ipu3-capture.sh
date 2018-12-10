#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2018, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# ipu3-capture.sh - Capture raw frames from cameras based on the Intel IPU3
#
# The scripts makes use of the following tools, which are expected to be
# executable from the system-wide path or from the local directory:
#
# - media-ctl (from v4l-utils git://linuxtv.org/v4l-utils.git)
# - raw2pnm (from nvt https://github.com/intel/nvt.git)
# - yavta (from git://git.ideasonboard.org/yavta.git)

# Locate the sensor entity
find_sensor() {
	local bus
	local sensor_name=$1

	bus=$(grep "$sensor_name" /sys/class/video4linux/v4l-subdev*/name | cut -d ' ' -f 2)
	if [[ -z $bus ]]; then
		echo "Sensor '$sensor_name' not found." >&2
		exit 1
	fi

	echo "$sensor_name $bus"
}

# Locate the media device
find_media_device() {
	local mdev

	for mdev in /dev/media* ; do
		media-ctl -d $mdev -p | grep -q "^driver[ \t]*ipu3-cio2$" && break
		mdev=
	done

	if [[ -z $mdev ]] ; then
		echo "IPU3 media device not found." >&2
		exit 1
	fi

	echo $mdev
}

# Locate the CSI2 and CIO2 and get the sensor format
parse_pipeline() {
	local cio2_queue
	local resolution
	local sensor=$1

	read cio2_queue bus_format sensor_size <<< $($mediactl -p | awk "
	/^- entity [0-9]*:/ {
		sensor=0;
	}

	/^- entity [0-9]*: $sensor/ {
		sensor=1;
	}

	/^[ \t]*(stream:0)?\[fmt:/ {
		if (sensor) {
			gsub(\".*fmt:\", \"\");
			gsub(\"[] ].*\", \"\");
			gsub(\"/\", \" \");
			format=\$0;
		}
	}

	/^[ \t]*->/ {
		if (sensor)
			cio2=substr(\$3, 0, 1);
	}

	END {
		print cio2 \" \" format;
	}
	")

	ipu3_csi2="ipu3-csi2 $cio2_queue"
	ipu3_capture="ipu3-cio2 $cio2_queue"

	sensor_width=$(echo $sensor_size | cut -d 'x' -f 1)
	sensor_height=$(echo $sensor_size | cut -d 'x' -f 2)

	echo "Using device $mdev with IPU3 CIO2 queue $cio2_queue"
}

# Configure the pipeline
configure_pipeline() {
	local format="fmt:$bus_format/$sensor_size"

	echo "Configuring pipeline for $sensor in $format"

	$mediactl -r

	$mediactl -l "\"$sensor\":0 -> \"$ipu3_csi2\":0[1]"
	$mediactl -l "\"$ipu3_csi2\":1 -> \"$ipu3_capture\":0[1]"

	$mediactl -V "\"$sensor\":0 [$format]"
	$mediactl -V "\"$ipu3_csi2\":1 [$format]"
}

# Capture frames
capture_frames() {
	local file_op
	local frame_count=$1
	local ipu3_format=IPU3_${bus_format/_1X10/}
	local save_file=$2

	if [[ $save_file -eq 1 ]]; then
		file_op="--file=/tmp/frame-#.bin"
	fi

	yavta -c$frame_count -n5 -I -f $ipu3_format -s $sensor_size $file_op \
		$($mediactl -e "$ipu3_capture")
}

# Convert captured files to ppm
convert_files() {
	local frame_count=$1
	local format=${bus_format/_1X10/}
	local padded_width=$(expr \( $sensor_width + 49 \) / 50 \* 50)

	echo "Converting ${sensor_width}x${sensor_height} (${padded_width}x${sensor_height})"

	for i in `seq -f '%06.0f' 0 $(($frame_count - 1))`; do
		ipu3-unpack /tmp/frame-$i.bin /tmp/frame-$i.raw
		raw2pnm -x$padded_width -y$sensor_height -f$format /tmp/frame-$i.raw /tmp/frame-$i.ppm
	done
}

# Print usage message
usage() {
	echo "Usage: $1 [options] sensor-name"
	echo "Supported options:"
	echo "-c,--count n      Number of frame to capture"
	echo "--no-save         Do not save captured frames to disk"
}

# Parse command line arguments
frame_count=10
save_file=1

while (( "$#" )) ; do
	case $1 in
	-c|--count)
		frame_count=$2
		shift 2
		;;
	--no-save)
		save_file=0
		shift
		;;
	-*)
		echo "Unsupported option $1" >&2
		usage $0
		exit 1
		;;
	*)
		break
		;;
	esac
done

if [[ $# -ne 1 ]] ; then
	usage $0
	exit 1
fi

sensor_name=$1

sensor=$(find_sensor $sensor_name) || exit
mdev=$(find_media_device) || exit
mediactl="media-ctl -d $mdev"

parse_pipeline $sensor
configure_pipeline
capture_frames $frame_count $save_file
[[ $save_file -eq 1 ]] && convert_files $frame_count
