# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2023, Raspberry Pi Ltd
#
# ctt_cac.py - CAC (Chromatic Aberration Correction) tuning tool

from PIL import Image
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm

from ctt_dots_locator import find_dots_locations


# This is the wrapper file that creates a JSON entry for you to append
# to your camera tuning file.
# It calculates the chromatic aberration at different points throughout
# the image and uses that to produce a martix that can then be used
# in the camera tuning files to correct this aberration.


def pprint_array(array):
    # Function to print the array in a tidier format
    array = array
    output = ""
    for i in range(len(array)):
        for j in range(len(array[0])):
            output += str(round(array[i, j], 2)) + ", "
        # Add the necessary indentation to the array
        output += "\n                   "
    # Cut off the end of the array (nicely formats it)
    return output[:-22]


def plot_shifts(red_shifts, blue_shifts):
    # If users want, they can pass a command line option to show the shifts on a graph
    # Can be useful to check that the functions are all working, and that the sample
    # images are doing the right thing
    Xs = np.array(red_shifts)[:, 0]
    Ys = np.array(red_shifts)[:, 1]
    Zs = np.array(red_shifts)[:, 2]
    Zs2 = np.array(red_shifts)[:, 3]
    Zs3 = np.array(blue_shifts)[:, 2]
    Zs4 = np.array(blue_shifts)[:, 3]

    fig, axs = plt.subplots(2, 2)
    ax = fig.add_subplot(2, 2, 1, projection='3d')
    ax.scatter(Xs, Ys, Zs, cmap=cm.jet, linewidth=0)
    ax.set_title('Red X Shift')
    ax = fig.add_subplot(2, 2, 2, projection='3d')
    ax.scatter(Xs, Ys, Zs2, cmap=cm.jet, linewidth=0)
    ax.set_title('Red Y Shift')
    ax = fig.add_subplot(2, 2, 3, projection='3d')
    ax.scatter(Xs, Ys, Zs3, cmap=cm.jet, linewidth=0)
    ax.set_title('Blue X Shift')
    ax = fig.add_subplot(2, 2, 4, projection='3d')
    ax.scatter(Xs, Ys, Zs4, cmap=cm.jet, linewidth=0)
    ax.set_title('Blue Y Shift')
    fig.tight_layout()
    plt.show()


def shifts_to_yaml(red_shift, blue_shift, image_dimensions, output_grid_size=9):
    # Convert the shifts to a numpy array for easier handling and initialise other variables
    red_shifts = np.array(red_shift)
    blue_shifts = np.array(blue_shift)
    # create a grid that's smaller than the output grid, which we then interpolate from to get the output values
    xrgrid = np.zeros((output_grid_size - 1, output_grid_size - 1))
    xbgrid = np.zeros((output_grid_size - 1, output_grid_size - 1))
    yrgrid = np.zeros((output_grid_size - 1, output_grid_size - 1))
    ybgrid = np.zeros((output_grid_size - 1, output_grid_size - 1))

    xrsgrid = []
    xbsgrid = []
    yrsgrid = []
    ybsgrid = []
    xg = np.zeros((output_grid_size - 1, output_grid_size - 1))
    yg = np.zeros((output_grid_size - 1, output_grid_size - 1))

    # Format the grids - numpy doesn't work for this, it wants a
    # nice uniformly spaced grid, which we don't know if we have yet, hence the rather mundane setup
    for x in range(output_grid_size - 1):
        xrsgrid.append([])
        yrsgrid.append([])
        xbsgrid.append([])
        ybsgrid.append([])
        for y in range(output_grid_size - 1):
            xrsgrid[x].append([])
            yrsgrid[x].append([])
            xbsgrid[x].append([])
            ybsgrid[x].append([])

    image_size = (image_dimensions[0], image_dimensions[1])
    gridxsize = image_size[0] / (output_grid_size - 1)
    gridysize = image_size[1] / (output_grid_size - 1)

    # Iterate through each dot, and it's shift values and put these into the correct grid location
    for red_shift in red_shifts:
        xgridloc = int(red_shift[0] / gridxsize)
        ygridloc = int(red_shift[1] / gridysize)
        xrsgrid[xgridloc][ygridloc].append(red_shift[2])
        yrsgrid[xgridloc][ygridloc].append(red_shift[3])

    for blue_shift in blue_shifts:
        xgridloc = int(blue_shift[0] / gridxsize)
        ygridloc = int(blue_shift[1] / gridysize)
        xbsgrid[xgridloc][ygridloc].append(blue_shift[2])
        ybsgrid[xgridloc][ygridloc].append(blue_shift[3])

    # Now calculate the average pixel shift for each square in the grid
    grid_incomplete = False
    for x in range(output_grid_size - 1):
        for y in range(output_grid_size - 1):
            if xrsgrid[x][y]:
                xrgrid[x, y] = np.mean(xrsgrid[x][y])
            else:
                grid_incomplete = True
            if yrsgrid[x][y]:
                yrgrid[x, y] = np.mean(yrsgrid[x][y])
            else:
                grid_incomplete = True
            if xbsgrid[x][y]:
                xbgrid[x, y] = np.mean(xbsgrid[x][y])
            else:
                grid_incomplete = True
            if ybsgrid[x][y]:
                ybgrid[x, y] = np.mean(ybsgrid[x][y])
            else:
                grid_incomplete = True

    if grid_incomplete:
        raise RuntimeError("\nERROR: CAC measurements do not span the image!"
                           "\nConsider using improved CAC images, or remove them entirely.\n")

    # Next, we start to interpolate the central points of the grid that gets passed to the tuning file
    input_grids = np.array([xrgrid, yrgrid, xbgrid, ybgrid])
    output_grids = np.zeros((4, output_grid_size, output_grid_size))

    # Interpolate the centre of the grid
    output_grids[:, 1:-1, 1:-1] = (input_grids[:, 1:, :-1] + input_grids[:, 1:, 1:] + input_grids[:, :-1, 1:] + input_grids[:, :-1, :-1]) / 4

    # Edge cases:
    output_grids[:, 1:-1, 0] = ((input_grids[:, :-1, 0] + input_grids[:, 1:, 0]) / 2 - output_grids[:, 1:-1, 1]) * 2 + output_grids[:, 1:-1, 1]
    output_grids[:, 1:-1, -1] = ((input_grids[:, :-1, 7] + input_grids[:, 1:, 7]) / 2 - output_grids[:, 1:-1, -2]) * 2 + output_grids[:, 1:-1, -2]
    output_grids[:, 0, 1:-1] = ((input_grids[:, 0, :-1] + input_grids[:, 0, 1:]) / 2 - output_grids[:, 1, 1:-1]) * 2 + output_grids[:, 1, 1:-1]
    output_grids[:, -1, 1:-1] = ((input_grids[:, 7, :-1] + input_grids[:, 7, 1:]) / 2 - output_grids[:, -2, 1:-1]) * 2 + output_grids[:, -2, 1:-1]

    # Corner Cases:
    output_grids[:, 0, 0] = (output_grids[:, 0, 1] - output_grids[:, 1, 1]) + (output_grids[:, 1, 0] - output_grids[:, 1, 1]) + output_grids[:, 1, 1]
    output_grids[:, 0, -1] = (output_grids[:, 0, -2] - output_grids[:, 1, -2]) + (output_grids[:, 1, -1] - output_grids[:, 1, -2]) + output_grids[:, 1, -2]
    output_grids[:, -1, 0] = (output_grids[:, -1, 1] - output_grids[:, -2, 1]) + (output_grids[:, -2, 0] - output_grids[:, -2, 1]) + output_grids[:, -2, 1]
    output_grids[:, -1, -1] = (output_grids[:, -2, -1] - output_grids[:, -2, -2]) + (output_grids[:, -1, -2] - output_grids[:, -2, -2]) + output_grids[:, -2, -2]

    # Below, we swap the x and the y coordinates, and also multiply by a factor of -1
    # This is due to the PiSP (standard) dimensions being flipped in comparison to
    # PIL image coordinate directions, hence why xr -> yr. Also, the shifts calculated are colour shifts,
    # and the PiSP block asks for the values it should shift by (hence the * -1, to convert from colour shift to a pixel shift)

    output_grid_yr, output_grid_xr, output_grid_yb, output_grid_xb = output_grids * -1
    return output_grid_xr, output_grid_yr, output_grid_xb, output_grid_yb


def analyse_dot(dot, dot_location=[0, 0]):
    # Scan through the dot, calculate the centroid of each colour channel by doing:
    # pixel channel brightness * distance from top left corner
    # Sum these, and divide by the sum of each channel's brightnesses to get a centroid for each channel
    red_channel = np.array(dot)[:, :, 0]
    y_num_pixels = len(red_channel[0])
    x_num_pixels = len(red_channel)
    yred_weight = np.sum(np.dot(red_channel, np.arange(y_num_pixels)))
    xred_weight = np.sum(np.dot(np.arange(x_num_pixels), red_channel))
    red_sum = np.sum(red_channel)

    green_channel = np.array(dot)[:, :, 1]
    ygreen_weight = np.sum(np.dot(green_channel, np.arange(y_num_pixels)))
    xgreen_weight = np.sum(np.dot(np.arange(x_num_pixels), green_channel))
    green_sum = np.sum(green_channel)

    blue_channel = np.array(dot)[:, :, 2]
    yblue_weight = np.sum(np.dot(blue_channel, np.arange(y_num_pixels)))
    xblue_weight = np.sum(np.dot(np.arange(x_num_pixels), blue_channel))
    blue_sum = np.sum(blue_channel)

    # We return this structure. It contains 2 arrays that contain:
    # the locations of the dot center, along with the channel shifts in the x and y direction:
    # [ [red_center_x, red_center_y, red_x_shift, red_y_shift], [blue_center_x, blue_center_y, blue_x_shift, blue_y_shift] ]

    return [[int(dot_location[0]) + int(len(dot) / 2), int(dot_location[1]) + int(len(dot[0]) / 2), xred_weight / red_sum - xgreen_weight / green_sum, yred_weight / red_sum - ygreen_weight / green_sum], [dot_location[0] + int(len(dot) / 2), dot_location[1] + int(len(dot[0]) / 2), xblue_weight / blue_sum - xgreen_weight / green_sum, yblue_weight / blue_sum - ygreen_weight / green_sum]]


def cac(Cam):
    filelist = Cam.imgs_cac

    Cam.log += '\nCAC analysing files: {}'.format(str(filelist))
    np.set_printoptions(precision=3)
    np.set_printoptions(suppress=True)

    # Create arrays to hold all the dots data and their colour offsets
    red_shift = []  # Format is: [[Dot Center X, Dot Center Y, x shift, y shift]]
    blue_shift = []
    # Iterate through the files
    # Multiple files is reccomended to average out the lens aberration through rotations
    for file in filelist:
        Cam.log += '\nCAC processing file'
        print("\n Processing file")
        # Read the raw RGB values
        rgb = file.rgb
        image_size = [file.h, file.w]  # Image size, X, Y
        # Create a colour copy of the RGB values to use later in the calibration
        imout = Image.new(mode="RGB", size=image_size)
        rgb_image = np.array(imout)
        # The rgb values need reshaping from a 1d array to a 3d array to be worked with easily
        rgb.reshape((image_size[0], image_size[1], 3))
        rgb_image = rgb

        # Pass the RGB image through to the dots locating program
        # Returns an array of the dots (colour rectangles around the dots), and an array of their locations
        print("Finding dots")
        Cam.log += '\nFinding dots'
        dots, dots_locations = find_dots_locations(rgb_image)

        # Now, analyse each dot. Work out the centroid of each colour channel, and use that to work out
        # by how far the chromatic aberration has shifted each channel
        Cam.log += '\nDots found: {}'.format(str(len(dots)))
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
    Cam.log += '\nCreating output grid'
    try:
        rx, ry, bx, by = shifts_to_yaml(red_shift, blue_shift, image_size)
    except RuntimeError as e:
        print(str(e))
        Cam.log += "\nCAC correction failed! CAC will not be enabled."
        return {}

    print("CAC correction complete!")
    Cam.log += '\nCAC correction complete!'

    # Give the JSON dict back to the main ctt program
    return {"strength": 1.0, "lut_rx": list(rx.round(2).reshape(81)), "lut_ry": list(ry.round(2).reshape(81)), "lut_bx": list(bx.round(2).reshape(81)), "lut_by": list(by.round(2).reshape(81))}
