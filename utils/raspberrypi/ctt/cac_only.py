#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2023, Raspberry Pi (Trading) Ltd.
#
# cac_only.py - cac tuning tool


# This file allows you to tune only the chromatic aberration correction
# Specify any number of files in the command line args, and it shall iterate through
# and generate an averaged cac table from all the input images, which you can then
# input into your tuning file.

# Takes .dng files produced by the camera modules of the dots grid and calculates the chromatic abberation of each dot.
# Then takes each dot, and works out where it was in the image, and uses that to output a tables of the shifts
# across the whole image.

from PIL import Image
import numpy as np
import rawpy
import sys
import getopt

from ctt_cac import *


def cac(filelist, output_filepath, plot_results=False):
    np.set_printoptions(precision=3)
    np.set_printoptions(suppress=True)

    # Create arrays to hold all the dots data and their colour offsets
    red_shift = []  # Format is: [[Dot Center X, Dot Center Y, x shift, y shift]]
    blue_shift = []
    # Iterate through the files
    # Multiple files is reccomended to average out the lens aberration through rotations
    for file in filelist:
        print("\n Processing file " + str(file))
        # Read the raw RGB values from the .dng file
        with rawpy.imread(file) as raw:
            rgb = raw.postprocess()
            sizes = (raw.sizes)

        image_size = [sizes[2], sizes[3]]  # Image size, X, Y
        # Create a colour copy of the RGB values to use later in the calibration
        imout = Image.new(mode="RGB", size=image_size)
        rgb_image = np.array(imout)
        # The rgb values need reshaping from a 1d array to a 3d array to be worked with easily
        rgb.reshape((image_size[0], image_size[1], 3))
        rgb_image = rgb

        # Pass the RGB image through to the dots locating program
        # Returns an array of the dots (colour rectangles around the dots), and an array of their locations
        print("Finding dots")
        dots, dots_locations = find_dots_locations(rgb_image)

        # Now, analyse each dot. Work out the centroid of each colour channel, and use that to work out
        # by how far the chromatic aberration has shifted each channel
        print('Dots found: ' + str(len(dots)))

        for dot, dot_location in zip(dots, dots_locations):
            if len(dot) > 0:
                if (dot_location[0] > 0) and (dot_location[1] > 0):
                    ret = analyse_dot(dot, dot_location)
                    red_shift.append(ret[0])
                    blue_shift.append(ret[1])

    # Take our arrays of red shifts and locations, push them through to be interpolated into a 9x9 matrix
    # for the CAC block to handle and then store these as a .json file to be added to the camera
    # tuning file
    print("\nCreating output grid")
    rx, ry, bx, by = shifts_to_yaml(red_shift, blue_shift, image_size)

    print("CAC correction complete!")

    # The json format that we then paste into the tuning file (manually)
    sample = '''
    {
        "rpi.cac" :
        {
            "strength": 1.0,
            "lut_rx" : [
                   rx_vals
            ],
            "lut_ry" : [
                   ry_vals
            ],
            "lut_bx" : [
                   bx_vals
            ],
            "lut_by" : [
                   by_vals
            ]
        }
    }
    '''

    # Below, may look incorrect, however, the PiSP (standard) dimensions are flipped in comparison to
    # PIL image coordinate directions, hence why xr -> yr. Also, the shifts calculated are colour shifts,
    # and the PiSP block asks for the values it should shift (hence the * -1, to convert from colour shift to a pixel shift)
    sample = sample.replace("rx_vals", pprint_array(ry * -1))
    sample = sample.replace("ry_vals", pprint_array(rx * -1))
    sample = sample.replace("bx_vals", pprint_array(by * -1))
    sample = sample.replace("by_vals", pprint_array(bx * -1))
    print("Successfully converted to JSON")
    f = open(str(output_filepath), "w+")
    f.write(sample)
    f.close()
    print("Successfully written to json file")
    '''
    If you wish to see a plot of the colour channel shifts, add the -p or --plots option
    Can be a quick way of validating if the data/dots you've got are good, or if you need to
    change some parameters/take some better images
    '''
    if plot_results:
        plot_shifts(red_shift, blue_shift)


if __name__ == "__main__":
    argv = sys.argv
    # Detect the input and output file paths
    arg_output = "output.json"
    arg_help = "{0} -i <input> -o <output> -p <plot results>".format(argv[0])
    opts, args = getopt.getopt(argv[1:], "hi:o:p", ["help", "input=", "output=", "plot"])

    output_location = 0
    input_location = 0
    filelist = []
    plot_results = False
    for i in range(len(argv)):
        if ("-h") in argv[i]:
            print(arg_help)  # print the help message
            sys.exit(2)
        if "-o" in argv[i]:
            output_location = i
        if ".dng" in argv[i]:
            filelist.append(argv[i])
        if "-p" in argv[i]:
            plot_results = True

    arg_output = argv[output_location + 1]
    cac(filelist, arg_output, plot_results)
