#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2019, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# rkisp-capture.sh - Capture processed frames from cameras based on the
# Rockchip ISP1
#
# The scripts makes use of the following tools, which are expected to be
# executable from the system-wide path or from the local directory:
#
# - media-ctl (from v4l-utils git://linuxtv.org/v4l-utils.git)
# - raw2rgbpnm (from git://git.retiisi.org.uk/~sailus/raw2rgbpnm.git)
# - yavta (from git://git.ideasonboard.org/yavta.git)

# Locate the sensor entity
find_sensor() {
	local bus
	local sensor_name=$1

	bus=$(grep "$sensor_name " /sys/class/video4linux/v4l-subdev*/name | cut -d ' ' -f 2)
	if [[ -z $bus ]]; then
		echo "Sensor '$sensor_name' not found." >&2
		exit 1
	fi

	echo "$sensor_name $bus"
}

# Locate the media device
find_media_device() {
	local mdev
	local name=$1

	for mdev in /dev/media* ; do
		media-ctl -d $mdev -p | grep -q "^driver[ \t]*$name$" && break
		mdev=
	done

	if [[ -z $mdev ]] ; then
		echo "$name media device not found." >&2
		exit 1
	fi

	echo $mdev
}

# Get the sensor format
get_sensor_format() {
	local format
	local sensor=$1

	format=$($mediactl --get-v4l2 "'$sensor':0" | sed 's/\[\([^ ]*\).*/\1/')
	sensor_mbus_code=$(echo $format | sed 's/fmt:\([A-Z0-9_]*\).*/\1/')
	sensor_size=$(echo $format | sed 's/[^\/]*\/\([0-9x]*\).*/\1/')

	echo "Capturing ${sensor_size} from sensor $sensor in ${sensor_mbus_code}"
}

# Configure the pipeline
configure_pipeline() {
	local format="fmt:$sensor_mbus_code/$sensor_size"
	local capture_mbus_code=$1
	local capture_size=$2

	echo "Configuring pipeline for $sensor in $format"

	$mediactl -r

	$mediactl -l "'$sensor':0 -> 'rockchip-sy-mipi-dphy':0 [1]"
	$mediactl -l "'rockchip-sy-mipi-dphy':1 -> 'rkisp1-isp-subdev':0 [1]"
	$mediactl -l "'rkisp1-isp-subdev':2 -> 'rkisp1_mainpath':0 [1]"

	$mediactl -V "\"$sensor\":0 [$format]"
	$mediactl -V "'rockchip-sy-mipi-dphy':1 [$format]"
	$mediactl -V "'rkisp1-isp-subdev':0 [$format crop:(0,0)/$sensor_size]"
	$mediactl -V "'rkisp1-isp-subdev':2 [fmt:$capture_mbus_code/$capture_size crop:(0,0)/$capture_size]"
}

# Capture frames
capture_frames() {
	local file_op
	local capture_format=$1
	local capture_size=$2
	local frame_count=$3
	local save_file=$4

	if [[ $save_file -eq 1 ]]; then
		file_op="--file=/tmp/frame-#.bin"
	fi

	yavta -c$frame_count -n5 -I -f $capture_format -s $capture_size \
		$file_op $($mediactl -e "rkisp1_mainpath")
}

# Convert captured files to ppm
convert_files() {
	local format=$1
	local size=$2
	local frame_count=$3

	echo "Converting ${frame_count} frames (${size})"

	for i in `seq 0 $(($frame_count - 1))`; do
		i=$(printf %06u $i)
		raw2rgbpnm -f $format -s $size /tmp/frame-$i.bin /tmp/frame-$i.ppm
	done
}

# Print usage message
usage() {
	echo "Usage: $1 [options] sensor-name"
	echo "Supported options:"
	echo "-c,--count n      Number of frame to capture"
	echo "--no-save         Do not save captured frames to disk"
	echo "-r, --raw         Capture RAW frames"
	echo "-s, --size wxh    Frame size"
}

# Parse command line arguments
capture_size=1024x768
frame_count=10
raw=false
save_file=1

while [[ $# -ne 0 ]] ; do
	case $1 in
	-c|--count)
		frame_count=$2
		shift 2
		;;
	--no-save)
		save_file=0
		shift
		;;

	-r|--raw)
		raw=true
		shift
		;;
	-s|--size)
		capture_size=$2
		shift 2
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

modprobe mipi_dphy_sy
modprobe video_rkisp1

sensor=$(find_sensor $sensor_name) || exit
mdev=$(find_media_device rkisp1) || exit
mediactl="media-ctl -d $mdev"

get_sensor_format "$sensor"
if [[ $raw == true ]] ; then
	capture_format=$(echo $sensor_mbus_code | sed 's/_[0-9X]$//')
	capture_mbus_code=$sensor_mbus_code
else
	capture_format=YUYV
	capture_mbus_code=YUYV8_2X8
fi

configure_pipeline $capture_mbus_code $capture_size
capture_frames $capture_format $capture_size $frame_count $save_file
[[ $save_file -eq 1 ]] && convert_files $capture_format $capture_size $frame_count
