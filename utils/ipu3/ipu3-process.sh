#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2018, Google Inc.
#
# Author: Laurent Pinchart <laurent.pinchart@ideasonboard.com>
#
# Process raw frames with the Intel IPU3
#
# The scripts makes use of the following tools, which are expected to be
# found in $PATH:
#
# - media-ctl (from v4l-utils git://linuxtv.org/v4l-utils.git)
# - raw2pnm (from nvt https://github.com/intel/nvt.git)
# - yavta (from git://git.ideasonboard.org/yavta.git)

imgu_entity="ipu3-imgu 0"

# Locate the media device
find_media_device() {
	local mdev

	for mdev in /dev/media* ; do
		media-ctl -d $mdev -p | grep -q "^driver[ \t]*ipu3-imgu$" && break
		mdev=
	done

	if [[ -z $mdev ]] ; then
		echo "IPU3 media device not found." >&2
		exit 1
	fi

	echo $mdev
}

# Configure the pipeline
configure_pipeline() {
	local enable_3a=1
	local enable_out=1
	local enable_vf=1
	local mode=0

	# Configure the links
	$mediactl -r
	$mediactl -l "\"$imgu_entity input\":0 -> \"$imgu_entity\":0[1]"
	$mediactl -l "\"$imgu_entity\":2 -> \"$imgu_entity output\":0[$enable_out]"
	$mediactl -l "\"$imgu_entity\":3 -> \"$imgu_entity viewfinder\":0[$enable_vf]"
	$mediactl -l "\"$imgu_entity\":4 -> \"$imgu_entity 3a stat\":0[$enable_3a]"

	# Select processing mode (0 for video, 1 for still image)
	yavta --no-query -w "0x009819c1 $mode" $($mediactl -e "$imgu_entity")

	# Set formats. The media bus code doesn't matter as it is ignored by the
	# driver. We should use the FIXED format, but media-ctl doesn't support
	# it.
	$mediactl -V "\"$imgu_entity\":0 [fmt:SBGGR10_1X10/$out_size crop:(0,0)/$in_size compose:(0,0)/$in_size]"
	$mediactl -V "\"$imgu_entity\":2 [fmt:SBGGR10_1X10/$out_size]"
	$mediactl -V "\"$imgu_entity\":3 [fmt:SBGGR10_1X10/$vf_size]"
	$mediactl -V "\"$imgu_entity\":4 [fmt:SBGGR10_1X10/$out_size]"
}

# Perform frame processing through the IMGU
process_frames() {
	configure_pipeline

	local yavta="yavta -n $nbufs -c$frame_count"

	# Save the main and viewfinder outputs to disk, capture and drop 3A
	# statistics. Sleep 500ms between each execution of yavta to keep the
	# stdout messages readable.
	$yavta -f $IMGU_OUT_PIXELFORMAT -s $out_size "-F$output_dir/frame-out-#.bin" \
		$($mediactl -e "$imgu_entity output") &
	sleep 0.5
	$yavta -f $IMGU_VF_PIXELFORMAT -s $vf_size "-F$output_dir/frame-vf-#.bin" \
		$($mediactl -e "$imgu_entity viewfinder") &
	sleep 0.5
	$yavta $($mediactl -e "$imgu_entity 3a stat") &
	sleep 0.5

	# Feed the IMGU input.
	$yavta -f $IMGU_IN_PIXELFORMAT -s $in_size "-F$in_file" \
		$($mediactl -e "$imgu_entity input")
}

# Convert captured files to ppm
convert_files() {
	local index=$1
	local type=$2
	local size=$3
	local format=$4

	local width=$(echo $size | awk -F 'x' '{print $1}')
	local height=$(echo $size | awk -F 'x' '{print $2}')
	local padded_width=$(expr $(expr $width + 63) / 64 \* 64)

	raw2pnm -x$padded_width -y$height -f$format \
		$output_dir/frame-$type-$index.bin \
		$output_dir/frame-$type-$index.ppm
}

run_test() {
	IMGU_IN_PIXELFORMAT=IPU3_SGRBG10
	IMGU_OUT_PIXELFORMAT=NV12
	IMGU_VF_PIXELFORMAT=NV12

	echo "==== Test ===="
	echo "input:  $in_file"
	echo "output: $IMGU_OUT_PIXELFORMAT/$out_size"
	echo "vf:     $IMGU_VF_PIXELFORMAT/$vf_size"

	process_frames

	for i in `seq -f '%06.0f' 0 $(($frame_count - 1))`; do
		convert_files $i out $out_size $IMGU_OUT_PIXELFORMAT
		convert_files $i vf $vf_size $IMGU_VF_PIXELFORMAT
	done
}

validate_size() {
	local size=$1
	local width=$(echo $size | awk -F 'x' '{print $1}')
	local height=$(echo $size | awk -F 'x' '{print $2}')

	[[ "x${size}" == "x${width}x${height}" ]]
}

# Print usage message
usage() {
	echo "Usage: $(basename $1) [options] <input-file>"
	echo "Supported options:"
	echo "--out size        output frame size (defaults to input size)"
	echo "--vf size         viewfinder frame size (defaults to input size)"
	echo ""
	echo "Where the input file name and size are"
	echo ""
	echo "input-file = prefix '-' width 'x' height '.' extension"
	echo "size = width 'x' height"
}

# Parse command line arguments
while (( "$#" )) ; do
	case $1 in
	--out)
		out_size=$2
		if ! validate_size $out_size ; then
			echo "Invalid size '$out_size'"
			usage $0
			exit 1
		fi
		shift 2
		;;
	--vf)
		vf_size=$2
		if ! validate_size $vf_size ; then
			echo "Invalid size '$vf_size'"
			usage $0
			exit 1
		fi
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

if [ $# != 1 ] ; then
	usage $0
	exit 1
fi

in_file=$1

# Parse the size from the input file name and perform minimal sanity
# checks.
in_size=$(echo $in_file | sed 's/.*-\([0-9]*\)x\([0-9]*\)\.[a-z0-9]*$/\1x\2/')
validate_size $in_size
if [[ $? != 0 ]] ; then
	echo "Invalid input file name $in_file" >&2
	usage $0
	exit 1
fi

out_size=${out_size:-$in_size}
vf_size=${vf_size:-$in_size}

mdev=$(find_media_device) || exit
mediactl="media-ctl -d $mdev"
echo "Using device $mdev"

output_dir="/tmp"
frame_count=5
nbufs=7
run_test
