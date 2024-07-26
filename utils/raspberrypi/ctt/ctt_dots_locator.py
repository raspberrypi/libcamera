# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2023, Raspberry Pi Ltd
#
# find_dots.py - Used by CAC algorithm to convert image to set of dots

'''
This file takes the black and white version of the image, along with
the color version. It then located the black dots on the image by
thresholding dark pixels.
In a rather fun way, the algorithm bounces around the thresholded area in a random path
We then use the maximum and minimum of these paths to determine the dot shape and size
This info is then used to return colored dots and locations back to the main file
'''

import numpy as np
import random
from PIL import Image, ImageEnhance, ImageFilter


def find_dots_locations(rgb_image, color_threshold=100, dots_edge_avoid=75, image_edge_avoid=10, search_path_length=500, grid_scan_step_size=10, logfile=open("log.txt", "a+")):
    # Initialise some starting variables
    pixels = Image.fromarray(rgb_image)
    pixels = pixels.convert("L")
    enhancer = ImageEnhance.Contrast(pixels)
    im_output = enhancer.enhance(1.4)
    # We smooth it slightly to make it easier for the dot recognition program to locate the dots
    im_output = im_output.filter(ImageFilter.GaussianBlur(radius=2))
    bw_image = np.array(im_output)

    location = [0, 0]
    dots = []
    dots_location = []
    # the program takes away the edges - we don't want a dot that is half a circle, the
    # centroids would all be wrong
    for x in range(dots_edge_avoid, len(bw_image) - dots_edge_avoid, grid_scan_step_size):
        for y in range(dots_edge_avoid, len(bw_image[0]) - dots_edge_avoid, grid_scan_step_size):
            location = [x, y]
            scrap_dot = False  # A variable used to make sure that this is a valid dot
            if (bw_image[location[0], location[1]] < color_threshold) and not (scrap_dot):
                heading = "south"  # Define a starting direction to move in
                coords = []
                for i in range(search_path_length):  # Creates a path of length `search_path_length`. This turns out to always be enough to work out the rough shape of the dot.
                    # Now make sure that the thresholded area doesn't come within 10 pixels of the edge of the image, ensures we capture all the CA
                    if ((image_edge_avoid < location[0] < len(bw_image) - image_edge_avoid) and (image_edge_avoid < location[1] < len(bw_image[0]) - image_edge_avoid)) and not (scrap_dot):
                        if heading == "south":
                            if bw_image[location[0] + 1, location[1]] < color_threshold:
                                # Here, notice it does not go south, but actually goes southeast
                                # This is crucial in ensuring that we make our way around the majority of the dot
                                location[0] = location[0] + 1
                                location[1] = location[1] + 1
                                heading = "south"
                            else:
                                # This happens when we reach a thresholded edge. We now randomly change direction and keep searching
                                dir = random.randint(1, 2)
                                if dir == 1:
                                    heading = "west"
                                if dir == 2:
                                    heading = "east"

                        if heading == "east":
                            if bw_image[location[0], location[1] + 1] < color_threshold:
                                location[1] = location[1] + 1
                                heading = "east"
                            else:
                                dir = random.randint(1, 2)
                                if dir == 1:
                                    heading = "north"
                                if dir == 2:
                                    heading = "south"

                        if heading == "west":
                            if bw_image[location[0], location[1] - 1] < color_threshold:
                                location[1] = location[1] - 1
                                heading = "west"
                            else:
                                dir = random.randint(1, 2)
                                if dir == 1:
                                    heading = "north"
                                if dir == 2:
                                    heading = "south"

                        if heading == "north":
                            if bw_image[location[0] - 1, location[1]] < color_threshold:
                                location[0] = location[0] - 1
                                heading = "north"
                            else:
                                dir = random.randint(1, 2)
                                if dir == 1:
                                    heading = "west"
                                if dir == 2:
                                    heading = "east"
                        # Log where our particle travels across the dot
                        coords.append([location[0], location[1]])
                    else:
                        scrap_dot = True  # We just don't have enough space around the dot, discard this one, and move on
                if not scrap_dot:
                    # get the size of the dot surrounding the dot
                    x_coords = np.array(coords)[:, 0]
                    y_coords = np.array(coords)[:, 1]
                    hsquaresize = max(list(x_coords)) - min(list(x_coords))
                    vsquaresize = max(list(y_coords)) - min(list(y_coords))
                    # Create the bounding coordinates of the rectangle surrounding the dot
                    # Program uses the dotsize + half of the dotsize to ensure we get all that color fringing
                    extra_space_factor = 0.45
                    top_left_x = (min(list(x_coords)) - int(hsquaresize * extra_space_factor))
                    btm_right_x = max(list(x_coords)) + int(hsquaresize * extra_space_factor)
                    top_left_y = (min(list(y_coords)) - int(vsquaresize * extra_space_factor))
                    btm_right_y = max(list(y_coords)) + int(vsquaresize * extra_space_factor)
                    # Overwrite the area of the dot to ensure we don't use it again
                    bw_image[top_left_x:btm_right_x, top_left_y:btm_right_y] = 255
                    # Add the color version of the dot to the list to send off, along with some coordinates.
                    dots.append(rgb_image[top_left_x:btm_right_x, top_left_y:btm_right_y])
                    dots_location.append([top_left_x, top_left_y])
                else:
                    # Dot was too close to the image border to be useable
                    pass
    return dots, dots_location
