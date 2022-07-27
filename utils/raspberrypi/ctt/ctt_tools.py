# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_tools.py - camera tuning tool miscellaneous

import time
import re
import binascii
import os
import cv2
import numpy as np
import imutils
import sys
import matplotlib.pyplot as plt
from sklearn import cluster as cluster
from sklearn.neighbors import NearestCentroid as get_centroids

"""
This file contains some useful tools, the details of which aren't important to
understanding of the code. They ar collated here to attempt to improve code
readability in the main files.
"""


"""
obtain config values, unless it doesnt exist, in which case pick default
Furthermore, it can check if the input is the correct type
"""
def get_config(dictt, key, default, ttype):
    try:
        val = dictt[key]
        if ttype == 'string':
            val = str(val)
        elif ttype == 'num':
            if 'int' not in str(type(val)):
                if 'float' not in str(type(val)):
                    raise ValueError
        elif ttype == 'dict':
            if not isinstance(val, dict):
                raise ValueError
        elif ttype == 'list':
            if not isinstance(val, list):
                raise ValueError
        elif ttype == 'bool':
            ttype = int(bool(ttype))
        else:
            val = dictt[key]
    except (KeyError, ValueError):
        val = default
    return val


"""
argument parser
"""
def parse_input():
    arguments = sys.argv[1:]
    if len(arguments) % 2 != 0:
        raise ArgError('\n\nERROR! Enter value for each arguent passed.')
    params = arguments[0::2]
    vals = arguments[1::2]
    args_dict = dict(zip(params, vals))
    json_output = get_config(args_dict, '-o', None, 'string')
    directory = get_config(args_dict, '-i', None, 'string')
    config = get_config(args_dict, '-c', None, 'string')
    log_path = get_config(args_dict, '-l', None, 'string')
    if directory is None:
        raise ArgError('\n\nERROR! No input directory given.')
    if json_output is None:
        raise ArgError('\n\nERROR! No output json given.')
    return json_output, directory, config, log_path


"""
custom arg and macbeth error class
"""
class ArgError(Exception):
    pass
class MacbethError(Exception):
    pass


"""
correlation function to quantify match
"""
def correlate(im1, im2):
    f1 = im1.flatten()
    f2 = im2.flatten()
    cor = np.corrcoef(f1, f2)
    return cor[0][1]


"""
get list of files from directory
"""
def get_photos(directory='photos'):
    filename_list = []
    for filename in os.listdir(directory):
        if 'jp' in filename or '.dng' in filename:
            filename_list.append(filename)
    return filename_list


"""
display image for debugging... read at your own risk...
"""
def represent(img, name='image'):
    # if type(img) == tuple or type(img) == list:
    #     for i in range(len(img)):
    #         name = 'image {}'.format(i)
    #         cv2.imshow(name, img[i])
    # else:
    #     cv2.imshow(name, img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    # return 0
    """
    code above displays using opencv, but this doesn't catch users pressing 'x'
    with their mouse to close the window....  therefore matplotlib is used....
    (thanks a lot opencv)
    """
    grid = plt.GridSpec(22, 1)
    plt.subplot(grid[:19, 0])
    plt.imshow(img, cmap='gray')
    plt.axis('off')
    plt.subplot(grid[21, 0])
    plt.title('press \'q\' to continue')
    plt.axis('off')
    plt.show()

    # f = plt.figure()
    # ax = f.add_subplot(211)
    # ax2 = f.add_subplot(122)
    # ax.imshow(img, cmap='gray')
    # ax.axis('off')
    # ax2.set_figheight(2)
    # ax2.title('press \'q\' to continue')
    # ax2.axis('off')
    # plt.show()


"""
reshape image to fixed width without distorting
returns image and scale factor
"""
def reshape(img, width):
    factor = width/img.shape[0]
    return cv2.resize(img, None, fx=factor, fy=factor), factor
