#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt.py - camera tuning tool

import os
import sys
from ctt_image_load import *
from ctt_ccm import *
from ctt_awb import *
from ctt_alsc import *
from ctt_lux import *
from ctt_noise import *
from ctt_geq import *
from ctt_pretty_print_json import pretty_print
import random
import json
import re

"""
This file houses the camera object, which is used to perform the calibrations.
The camera object houses all the calibration images as attributes in two lists:
    - imgs (macbeth charts)
    - imgs_alsc (alsc correction images)
Various calibrations are methods of the camera object, and the output is stored
in a dictionary called self.json.
Once all the caibration has been completed, the Camera.json is written into a
json file.
The camera object initialises its json dictionary by reading from a pre-written
blank json file. This has been done to avoid reproducing the entire json file
in the code here, thereby avoiding unecessary clutter.
"""


"""
Get the colour and lux values from the strings of each inidvidual image
"""
def get_col_lux(string):
    """
    Extract colour and lux values from filename
    """
    col = re.search(r'([0-9]+)[kK](\.(jpg|jpeg|brcm|dng)|_.*\.(jpg|jpeg|brcm|dng))$', string)
    lux = re.search(r'([0-9]+)[lL](\.(jpg|jpeg|brcm|dng)|_.*\.(jpg|jpeg|brcm|dng))$', string)
    try:
        col = col.group(1)
    except AttributeError:
        """
        Catch error if images labelled incorrectly and pass reasonable defaults
        """
        return None, None
    try:
        lux = lux.group(1)
    except AttributeError:
        """
        Catch error if images labelled incorrectly and pass reasonable defaults
        Still returns colour if that has been found.
        """
        return col, None
    return int(col), int(lux)


"""
Camera object that is the backbone of the tuning tool.
Input is the desired path of the output json.
"""
class Camera:
    def __init__(self, jfile):
        self.path = os.path.dirname(os.path.expanduser(__file__)) + '/'
        if self.path == '/':
            self.path = ''
        self.imgs = []
        self.imgs_alsc = []
        self.log = 'Log created : ' + time.asctime(time.localtime(time.time()))
        self.log_separator = '\n'+'-'*70+'\n'
        self.jf = jfile
        """
        initial json dict populated by uncalibrated values
        """
        self.json = {
            "rpi.black_level": {
                "black_level": 4096
            },
            "rpi.dpc": {
            },
            "rpi.lux": {
                "reference_shutter_speed": 10000,
                "reference_gain": 1,
                "reference_aperture": 1.0
            },
            "rpi.noise": {
            },
            "rpi.geq": {
            },
            "rpi.sdn": {
            },
            "rpi.awb": {
                "priors": [
                    {"lux": 0, "prior": [2000, 1.0, 3000, 0.0, 13000, 0.0]},
                    {"lux": 800, "prior": [2000, 0.0, 6000, 2.0, 13000, 2.0]},
                    {"lux": 1500, "prior": [2000, 0.0, 4000, 1.0, 6000, 6.0, 6500, 7.0, 7000, 1.0, 13000, 1.0]}
                ],
                "modes": {
                    "auto": {"lo": 2500, "hi": 8000},
                    "incandescent": {"lo": 2500, "hi": 3000},
                    "tungsten": {"lo": 3000, "hi": 3500},
                    "fluorescent": {"lo": 4000, "hi": 4700},
                    "indoor": {"lo": 3000, "hi": 5000},
                    "daylight": {"lo": 5500, "hi": 6500},
                    "cloudy": {"lo": 7000, "hi": 8600}
                },
                "bayes": 1
            },
            "rpi.agc": {
                "metering_modes": {
                    "centre-weighted": {
                        "weights": [3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0]
                    },
                    "spot": {
                        "weights": [2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
                    },
                    "matrix": {
                        "weights": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
                    }
                },
                "exposure_modes": {
                    "normal": {
                        "shutter": [100, 10000, 30000, 60000, 120000],
                        "gain": [1.0, 2.0, 4.0, 6.0, 6.0]
                    },
                    "short": {
                        "shutter": [100, 5000, 10000, 20000, 120000],
                        "gain": [1.0, 2.0, 4.0, 6.0, 6.0]
                    }
                },
                "constraint_modes": {
                    "normal": [
                        {"bound": "LOWER", "q_lo": 0.98, "q_hi": 1.0, "y_target": [0, 0.5, 1000, 0.5]}
                    ],
                    "highlight": [
                        {"bound": "LOWER", "q_lo": 0.98, "q_hi": 1.0, "y_target": [0, 0.5, 1000, 0.5]},
                        {"bound": "UPPER", "q_lo": 0.98, "q_hi": 1.0, "y_target": [0, 0.8, 1000, 0.8]}
                    ]
                },
                "y_target": [0, 0.16, 1000, 0.165, 10000, 0.17]
            },
            "rpi.alsc": {
                'omega': 1.3,
                'n_iter': 100,
                'luminance_strength': 0.7,
            },
            "rpi.contrast": {
                "ce_enable": 1,
                "gamma_curve": [
                    0,     0,
                    1024,  5040,
                    2048,  9338,
                    3072,  12356,
                    4096,  15312,
                    5120,  18051,
                    6144,  20790,
                    7168,  23193,
                    8192,  25744,
                    9216,  27942,
                    10240, 30035,
                    11264, 32005,
                    12288, 33975,
                    13312, 35815,
                    14336, 37600,
                    15360, 39168,
                    16384, 40642,
                    18432, 43379,
                    20480, 45749,
                    22528, 47753,
                    24576, 49621,
                    26624, 51253,
                    28672, 52698,
                    30720, 53796,
                    32768, 54876,
                    36864, 57012,
                    40960, 58656,
                    45056, 59954,
                    49152, 61183,
                    53248, 62355,
                    57344, 63419,
                    61440, 64476,
                    65535, 65535
                ]
            },
            "rpi.ccm": {
            },
            "rpi.sharpen": {
            }
        }

    """
    Perform colour correction calibrations by comparing macbeth patch colours
    to standard macbeth chart colours.
    """
    def ccm_cal(self, do_alsc_colour):
        if 'rpi.ccm' in self.disable:
            return 1
        print('\nStarting CCM calibration')
        self.log_new_sec('CCM')
        """
        if image is greyscale then CCm makes no sense
        """
        if self.grey:
            print('\nERROR: Can\'t do CCM on greyscale image!')
            self.log += '\nERROR: Cannot perform CCM calibration '
            self.log += 'on greyscale image!\nCCM aborted!'
            del self.json['rpi.ccm']
            return 0
        a = time.time()
        """
        Check if alsc tables have been generated, if not then do ccm without
        alsc
        """
        if ("rpi.alsc" not in self.disable) and do_alsc_colour:
            """
            case where ALSC colour has been done, so no errors should be
            expected...
            """
            try:
                cal_cr_list = self.json['rpi.alsc']['calibrations_Cr']
                cal_cb_list = self.json['rpi.alsc']['calibrations_Cb']
                self.log += '\nALSC tables found successfully'
            except KeyError:
                cal_cr_list, cal_cb_list = None, None
                print('WARNING! No ALSC tables found for CCM!')
                print('Performing CCM calibrations without ALSC correction...')
                self.log += '\nWARNING: No ALSC tables found.\nCCM calibration '
                self.log += 'performed without ALSC correction...'
        else:
            """
            case where config options result in CCM done without ALSC colour tables
            """
            cal_cr_list, cal_cb_list = None, None
            self.log += '\nWARNING: No ALSC tables found.\nCCM calibration '
            self.log += 'performed without ALSC correction...'

        """
        Do CCM calibration
        """
        try:
            ccms = ccm(self, cal_cr_list, cal_cb_list)
        except ArithmeticError:
            print('ERROR: Matrix is singular!\nTake new pictures and try again...')
            self.log += '\nERROR: Singular matrix encountered during fit!'
            self.log += '\nCCM aborted!'
            return 1
        """
        Write output to json
        """
        self.json['rpi.ccm']['ccms'] = ccms
        self.log += '\nCCM calibration written to json file'
        print('Finished CCM calibration')

    """
    Auto white balance calibration produces a colour curve for
    various colour temperatures, as well as providing a maximum 'wiggle room'
    distance from this curve (transverse_neg/pos).
    """
    def awb_cal(self, greyworld, do_alsc_colour):
        if 'rpi.awb' in self.disable:
            return 1
        print('\nStarting AWB calibration')
        self.log_new_sec('AWB')
        """
        if image is greyscale then AWB makes no sense
        """
        if self.grey:
            print('\nERROR: Can\'t do AWB on greyscale image!')
            self.log += '\nERROR: Cannot perform AWB calibration '
            self.log += 'on greyscale image!\nAWB aborted!'
            del self.json['rpi.awb']
            return 0
        """
        optional set greyworld (e.g. for noir cameras)
        """
        if greyworld:
            self.json['rpi.awb']['bayes'] = 0
            self.log += '\nGreyworld set'
        """
        Check if alsc tables have been generated, if not then do awb without
        alsc correction
        """
        if ("rpi.alsc" not in self.disable) and do_alsc_colour:
            try:
                cal_cr_list = self.json['rpi.alsc']['calibrations_Cr']
                cal_cb_list = self.json['rpi.alsc']['calibrations_Cb']
                self.log += '\nALSC tables found successfully'
            except KeyError:
                cal_cr_list, cal_cb_list = None, None
                print('ERROR, no ALSC calibrations found for AWB')
                print('Performing AWB without ALSC tables')
                self.log += '\nWARNING: No ALSC tables found.\nAWB calibration '
                self.log += 'performed without ALSC correction...'
        else:
            cal_cr_list, cal_cb_list = None, None
            self.log += '\nWARNING: No ALSC tables found.\nAWB calibration '
            self.log += 'performed without ALSC correction...'
        """
        call calibration function
        """
        plot = "rpi.awb" in self.plot
        awb_out = awb(self, cal_cr_list, cal_cb_list, plot)
        ct_curve, transverse_neg, transverse_pos = awb_out
        """
        write output to json
        """
        self.json['rpi.awb']['ct_curve'] = ct_curve
        self.json['rpi.awb']['sensitivity_r'] = 1.0
        self.json['rpi.awb']['sensitivity_b'] = 1.0
        self.json['rpi.awb']['transverse_pos'] = transverse_pos
        self.json['rpi.awb']['transverse_neg'] = transverse_neg
        self.log += '\nAWB calibration written to json file'
        print('Finished AWB calibration')

    """
    Auto lens shading correction completely mitigates the effects of lens shading for ech
    colour channel seperately, and then partially corrects for vignetting.
    The extent of the correction depends on the 'luminance_strength' parameter.
    """
    def alsc_cal(self, luminance_strength, do_alsc_colour):
        if 'rpi.alsc' in self.disable:
            return 1
        print('\nStarting ALSC calibration')
        self.log_new_sec('ALSC')
        """
        check if alsc images have been taken
        """
        if len(self.imgs_alsc) == 0:
            print('\nError:\nNo alsc calibration images found')
            self.log += '\nERROR: No ALSC calibration images found!'
            self.log += '\nALSC calibration aborted!'
            return 1
        self.json['rpi.alsc']['luminance_strength'] = luminance_strength
        if self.grey and do_alsc_colour:
            print('Greyscale camera so only luminance_lut calculated')
            do_alsc_colour = False
            self.log += '\nWARNING: ALSC colour correction cannot be done on '
            self.log += 'greyscale image!\nALSC colour corrections forced off!'
        """
        call calibration function
        """
        plot = "rpi.alsc" in self.plot
        alsc_out = alsc_all(self, do_alsc_colour, plot)
        cal_cr_list, cal_cb_list, luminance_lut, av_corn = alsc_out
        """
        write ouput to json and finish if not do_alsc_colour
        """
        if not do_alsc_colour:
            self.json['rpi.alsc']['luminance_lut'] = luminance_lut
            self.json['rpi.alsc']['n_iter'] = 0
            self.log += '\nALSC calibrations written to json file'
            self.log += '\nNo colour calibrations performed'
            print('Finished ALSC calibrations')
            return 1

        self.json['rpi.alsc']['calibrations_Cr'] = cal_cr_list
        self.json['rpi.alsc']['calibrations_Cb'] = cal_cb_list
        self.json['rpi.alsc']['luminance_lut'] = luminance_lut
        self.log += '\nALSC colour and luminance tables written to json file'

        """
        The sigmas determine the strength of the adaptive algorithm, that
        cleans up any lens shading that has slipped through the alsc. These are
        determined by measuring a 'worst-case' difference between two alsc tables
        that are adjacent in colour space. If, however, only one colour
        temperature has been provided, then this difference can not be computed
        as only one table is available.
        To determine the sigmas you would have to estimate the error of an alsc
        table with only the image it was taken on as a check. To avoid circularity,
        dfault exaggerated sigmas are used, which can result in too much alsc and
        is therefore not advised.
        In general, just take another alsc picture at another colour temperature!
        """

        if len(self.imgs_alsc) == 1:
            self.json['rpi.alsc']['sigma'] = 0.005
            self.json['rpi.alsc']['sigma_Cb'] = 0.005
            print('\nWarning:\nOnly one alsc calibration found'
                  '\nStandard sigmas used for adaptive algorithm.')
            print('Finished ALSC calibrations')
            self.log += '\nWARNING: Only one colour temperature found in '
            self.log += 'calibration images.\nStandard sigmas used for adaptive '
            self.log += 'algorithm!'
            return 1

        """
        obtain worst-case scenario residual sigmas
        """
        sigma_r, sigma_b = get_sigma(self, cal_cr_list, cal_cb_list)
        """
        write output to json
        """
        self.json['rpi.alsc']['sigma'] = np.round(sigma_r, 5)
        self.json['rpi.alsc']['sigma_Cb'] = np.round(sigma_b, 5)
        self.log += '\nCalibrated sigmas written to json file'
        print('Finished ALSC calibrations')

    """
    Green equalisation fixes problems caused by discrepancies in green
    channels. This is done by measuring the effect on macbeth chart patches,
    which ideally would have the same green values throughout.
    An upper bound linear model is fit, fixing a threshold for the green
    differences that are corrected.
    """
    def geq_cal(self):
        if 'rpi.geq' in self.disable:
            return 1
        print('\nStarting GEQ calibrations')
        self.log_new_sec('GEQ')
        """
        perform calibration
        """
        plot = 'rpi.geq' in self.plot
        slope, offset = geq_fit(self, plot)
        """
        write output to json
        """
        self.json['rpi.geq']['offset'] = offset
        self.json['rpi.geq']['slope'] = slope
        self.log += '\nGEQ calibrations written to json file'
        print('Finished GEQ calibrations')

    """
    Lux calibrations allow the lux level of a scene to be estimated by a ratio
    calculation. Lux values are used in the pipeline for algorithms such as AGC
    and AWB
    """
    def lux_cal(self):
        if 'rpi.lux' in self.disable:
            return 1
        print('\nStarting LUX calibrations')
        self.log_new_sec('LUX')
        """
        The lux calibration is done on a single image. For best effects, the
        image with lux level closest to 1000 is chosen.
        """
        luxes = [Img.lux for Img in self.imgs]
        argmax = luxes.index(min(luxes, key=lambda l: abs(1000-l)))
        Img = self.imgs[argmax]
        self.log += '\nLux found closest to 1000: {} lx'.format(Img.lux)
        self.log += '\nImage used: ' + Img.name
        if Img.lux < 50:
            self.log += '\nWARNING: Low lux could cause inaccurate calibrations!'
        """
        do calibration
        """
        lux_out, shutter_speed, gain = lux(self, Img)
        """
        write output to json
        """
        self.json['rpi.lux']['reference_shutter_speed'] = shutter_speed
        self.json['rpi.lux']['reference_gain'] = gain
        self.json['rpi.lux']['reference_lux'] = Img.lux
        self.json['rpi.lux']['reference_Y'] = lux_out
        self.log += '\nLUX calibrations written to json file'
        print('Finished LUX calibrations')

    """
    Noise alibration attempts to describe the noise profile of the sensor. The
    calibration is run on macbeth images and the final output is taken as the average
    """
    def noise_cal(self):
        if 'rpi.noise' in self.disable:
            return 1
        print('\nStarting NOISE calibrations')
        self.log_new_sec('NOISE')
        """
        run calibration on all images and sort by slope.
        """
        plot = "rpi.noise" in self.plot
        noise_out = sorted([noise(self, Img, plot) for Img in self.imgs], key=lambda x: x[0])
        self.log += '\nFinished processing images'
        """
        take the average of the interquartile
        """
        length = len(noise_out)
        noise_out = np.mean(noise_out[length//4:1+3*length//4], axis=0)
        self.log += '\nAverage noise profile: constant = {} '.format(int(noise_out[1]))
        self.log += 'slope = {:.3f}'.format(noise_out[0])
        """
        write to json
        """
        self.json['rpi.noise']['reference_constant'] = int(noise_out[1])
        self.json['rpi.noise']['reference_slope'] = round(noise_out[0], 3)
        self.log += '\nNOISE calibrations written to json'
        print('Finished NOISE calibrations')

    """
    Removes json entries that are turned off
    """
    def json_remove(self, disable):
        self.log_new_sec('Disabling Options', cal=False)
        if len(self.disable) == 0:
            self.log += '\nNothing disabled!'
            return 1
        for key in disable:
            try:
                del self.json[key]
                self.log += '\nDisabled: ' + key
            except KeyError:
                self.log += '\nERROR: ' + key + ' not found!'
    """
    writes the json dictionary to the raw json file then make pretty
    """
    def write_json(self):
        """
        Write json dictionary to file using our version 2 format
        """

        out_json = {
            "version": 2.0,
            'target': 'bcm2835',
            "algorithms": [{name: data} for name, data in self.json.items()],
        }

        with open(self.jf, 'w') as f:
            f.write(pretty_print(out_json))

    """
    add a new section to the log file
    """
    def log_new_sec(self, section, cal=True):
        self.log += '\n'+self.log_separator
        self.log += section
        if cal:
            self.log += ' Calibration'
        self.log += self.log_separator

    """
    write script arguments to log file
    """
    def log_user_input(self, json_output, directory, config, log_output):
        self.log_new_sec('User Arguments', cal=False)
        self.log += '\nJson file output: ' + json_output
        self.log += '\nCalibration images directory: ' + directory
        if config is None:
            self.log += '\nNo configuration file input... using default options'
        elif config is False:
            self.log += '\nWARNING: Invalid configuration file path...'
            self.log += ' using default options'
        elif config is True:
            self.log += '\nWARNING: Invalid syntax in configuration file...'
            self.log += ' using default options'
        else:
            self.log += '\nConfiguration file: ' + config
        if log_output is None:
            self.log += '\nNo log file path input... using default: ctt_log.txt'
        else:
            self.log += '\nLog file output: ' + log_output

        # if log_output

    """
    write log file
    """
    def write_log(self, filename):
        if filename is None:
            filename = 'ctt_log.txt'
        self.log += '\n' + self.log_separator
        with open(filename, 'w') as logfile:
            logfile.write(self.log)

    """
    Add all images from directory, pass into relevant list of images and
    extrace lux and temperature values.
    """
    def add_imgs(self, directory, mac_config, blacklevel=-1):
        self.log_new_sec('Image Loading', cal=False)
        img_suc_msg = 'Image loaded successfully!'
        print('\n\nLoading images from '+directory)
        self.log += '\nDirectory: ' + directory
        """
        get list of files
        """
        filename_list = get_photos(directory)
        print("Files found: {}".format(len(filename_list)))
        self.log += '\nFiles found: {}'.format(len(filename_list))
        """
        iterate over files
        """
        filename_list.sort()
        for filename in filename_list:
            address = directory + filename
            print('\nLoading image: '+filename)
            self.log += '\n\nImage: ' + filename
            """
            obtain colour and lux value
            """
            col, lux = get_col_lux(filename)
            """
            Check if image is an alsc calibration image
            """
            if 'alsc' in filename:
                Img = load_image(self, address, mac=False)
                self.log += '\nIdentified as an ALSC image'
                """
                check if imagae data has been successfully unpacked
                """
                if Img == 0:
                    print('\nDISCARDED')
                    self.log += '\nImage discarded!'
                    continue
                    """
                check that image colour temperature has been successfuly obtained
                """
                elif col is not None:
                    """
                    if successful, append to list and continue to next image
                    """
                    Img.col = col
                    Img.name = filename
                    self.log += '\nColour temperature: {} K'.format(col)
                    self.imgs_alsc.append(Img)
                    if blacklevel != -1:
                        Img.blacklevel_16 = blacklevel
                    print(img_suc_msg)
                    continue
                else:
                    print('Error! No colour temperature found!')
                    self.log += '\nWARNING: Error reading colour temperature'
                    self.log += '\nImage discarded!'
                    print('DISCARDED')
            else:
                self.log += '\nIdentified as macbeth chart image'
                """
                if image isn't an alsc correction then it must have a lux and a
                colour temperature value to be useful
                """
                if lux is None:
                    print('DISCARDED')
                    self.log += '\nWARNING: Error reading lux value'
                    self.log += '\nImage discarded!'
                    continue
                Img = load_image(self, address, mac_config)
                """
                check that image data has been successfuly unpacked
                """
                if Img == 0:
                    print('DISCARDED')
                    self.log += '\nImage discarded!'
                    continue
                else:
                    """
                    if successful, append to list and continue to next image
                    """
                    Img.col, Img.lux = col, lux
                    Img.name = filename
                    self.log += '\nColour temperature: {} K'.format(col)
                    self.log += '\nLux value: {} lx'.format(lux)
                    if blacklevel != -1:
                        Img.blacklevel_16 = blacklevel
                    print(img_suc_msg)
                    self.imgs.append(Img)

        print('\nFinished loading images')

    """
    Check that usable images have been found
    Possible errors include:
        - no macbeth chart
        - incorrect filename/extension
        - images from different cameras
    """
    def check_imgs(self, macbeth=True):
        self.log += '\n\nImages found:'
        self.log += '\nMacbeth : {}'.format(len(self.imgs))
        self.log += '\nALSC : {} '.format(len(self.imgs_alsc))
        self.log += '\n\nCamera metadata'
        """
        check usable images found
        """
        if len(self.imgs) == 0 and macbeth:
            print('\nERROR: No usable macbeth chart images found')
            self.log += '\nERROR: No usable macbeth chart images found'
            return 0
        elif len(self.imgs) == 0 and len(self.imgs_alsc) == 0:
            print('\nERROR: No usable images found')
            self.log += '\nERROR: No usable images found'
            return 0
        """
        Double check that every image has come from the same camera...
        """
        all_imgs = self.imgs + self.imgs_alsc
        camNames = list(set([Img.camName for Img in all_imgs]))
        patterns = list(set([Img.pattern for Img in all_imgs]))
        sigbitss = list(set([Img.sigbits for Img in all_imgs]))
        blacklevels = list(set([Img.blacklevel_16 for Img in all_imgs]))
        sizes = list(set([(Img.w, Img.h) for Img in all_imgs]))

        if len(camNames) == 1 and len(patterns) == 1 and len(sigbitss) == 1 and \
           len(blacklevels) == 1 and len(sizes) == 1:
            self.grey = (patterns[0] == 128)
            self.blacklevel_16 = blacklevels[0]
            self.log += '\nName: {}'.format(camNames[0])
            self.log += '\nBayer pattern case: {}'.format(patterns[0])
            if self.grey:
                self.log += '\nGreyscale camera identified'
            self.log += '\nSignificant bits: {}'.format(sigbitss[0])
            self.log += '\nBlacklevel: {}'.format(blacklevels[0])
            self.log += '\nImage size: w = {} h = {}'.format(sizes[0][0], sizes[0][1])
            return 1
        else:
            print('\nERROR: Images from different cameras')
            self.log += '\nERROR: Images are from different cameras'
            return 0


def run_ctt(json_output, directory, config, log_output, alsc_only=False):
    """
    check input files are jsons
    """
    if json_output[-5:] != '.json':
        raise ArgError('\n\nError: Output must be a json file!')
    if config is not None:
        """
        check if config file is actually a json
        """
        if config[-5:] != '.json':
            raise ArgError('\n\nError: Config file must be a json file!')
        """
        read configurations
        """
        try:
            with open(config, 'r') as config_json:
                configs = json.load(config_json)
        except FileNotFoundError:
            configs = {}
            config = False
        except json.decoder.JSONDecodeError:
            configs = {}
            config = True

    else:
        configs = {}
    """
    load configurations from config file, if not given then set default
    """
    disable = get_config(configs, "disable", [], 'list')
    plot = get_config(configs, "plot", [], 'list')
    awb_d = get_config(configs, "awb", {}, 'dict')
    greyworld = get_config(awb_d, "greyworld", 0, 'bool')
    alsc_d = get_config(configs, "alsc", {}, 'dict')
    do_alsc_colour = get_config(alsc_d, "do_alsc_colour", 1, 'bool')
    luminance_strength = get_config(alsc_d, "luminance_strength", 0.5, 'num')
    blacklevel = get_config(configs, "blacklevel", -1, 'num')
    macbeth_d = get_config(configs, "macbeth", {}, 'dict')
    mac_small = get_config(macbeth_d, "small", 0, 'bool')
    mac_show = get_config(macbeth_d, "show", 0, 'bool')
    mac_config = (mac_small, mac_show)

    if blacklevel < -1 or blacklevel >= 2**16:
        print('\nInvalid blacklevel, defaulted to 64')
        blacklevel = -1

    if luminance_strength < 0 or luminance_strength > 1:
        print('\nInvalid luminance_strength strength, defaulted to 0.5')
        luminance_strength = 0.5

    """
    sanitise directory path
    """
    if directory[-1] != '/':
        directory += '/'
    """
    initialise tuning tool and load images
    """
    try:
        Cam = Camera(json_output)
        Cam.log_user_input(json_output, directory, config, log_output)
        if alsc_only:
            disable = set(Cam.json.keys()).symmetric_difference({"rpi.alsc"})
        Cam.disable = disable
        Cam.plot = plot
        Cam.add_imgs(directory, mac_config, blacklevel)
    except FileNotFoundError:
        raise ArgError('\n\nError: Input image directory not found!')

    """
    preform calibrations as long as check_imgs returns True
    If alsc is activated then it must be done before awb and ccm since the alsc
    tables are used in awb and ccm calibrations
    ccm also technically does an awb but it measures this from the macbeth
    chart in the image rather than using calibration data
    """
    if Cam.check_imgs(macbeth=not alsc_only):
        if not alsc_only:
            Cam.json['rpi.black_level']['black_level'] = Cam.blacklevel_16
        Cam.json_remove(disable)
        print('\nSTARTING CALIBRATIONS')
        Cam.alsc_cal(luminance_strength, do_alsc_colour)
        Cam.geq_cal()
        Cam.lux_cal()
        Cam.noise_cal()
        Cam.awb_cal(greyworld, do_alsc_colour)
        Cam.ccm_cal(do_alsc_colour)
        print('\nFINISHED CALIBRATIONS')
        Cam.write_json()
        Cam.write_log(log_output)
        print('\nCalibrations written to: '+json_output)
        if log_output is None:
            log_output = 'ctt_log.txt'
        print('Log file written to: '+log_output)
        pass
    else:
        Cam.write_log(log_output)


if __name__ == '__main__':
    """
    initialise calibration
    """
    if len(sys.argv) == 1:
        print("""
    Pisp Camera Tuning Tool version 1.0

    Required Arguments:
    '-i' : Calibration image directory.
    '-o' : Name of output json file.

    Optional Arguments:
    '-c' : Config file for the CTT. If not passed, default parameters used.
    '-l' : Name of output log file. If not passed, 'ctt_log.txt' used.
              """)
        quit(0)
    else:
        """
        parse input arguments
        """
        json_output, directory, config, log_output = parse_input()
        run_ctt(json_output, directory, config, log_output)
