# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# camera tuning tool for CCM (colour correction matrix)

import logging

import numpy as np
from scipy.optimize import minimize

from . import ctt_colors as colors
from .image import Image
from .ctt_awb import get_alsc_patches
from .utils import visualise_macbeth_chart

logger = logging.getLogger(__name__)

"""
takes 8-bit macbeth chart values, degammas and returns 16 bit
"""

'''
This program has many options from which to derive the color matrix from.
The first is average. This minimises the average delta E across all patches of
the macbeth chart. Testing across all cameras yeilded this as the most color
accurate and vivid. Other options are avalible however.
Maximum minimises the maximum Delta E of the patches. It iterates through till
a minimum maximum is found (so that there is
not one patch that deviates wildly.)
This yields generally good results but overall the colors are less accurate
Have a fiddle with maximum and see what you think.
The final option allows you to select the patches for which to average across.
This means that you can bias certain patches, for instance if you want the
reds to be more accurate.
'''

matrix_selection_types = ["average", "maximum", "patches"]
typenum = 0  # select from array above, 0 = average, 1 = maximum, 2 = patches
test_patches = [1, 2, 5, 8, 9, 12, 14]

'''
Enter patches to test for. Can also be entered twice if you
would like twice as much bias on one patch.
'''


def degamma(x):
    x = x / ((2 ** 8) - 1)  # takes 255 and scales it down to one
    x = np.where(x < 0.04045, x / 12.92, ((x + 0.055) / 1.055) ** 2.4)
    x = x * ((2 ** 16) - 1)  # takes one and scales up to 65535, 16 bit color
    return x


def gamma(x):
    # Take 3 long array of color values and gamma them
    return [((colour / 255) ** (1 / 2.4) * 1.055 - 0.055) * 255 for colour in x]


"""
FInds colour correction matrices for list of images
"""


def ccm(imgs, cal_cr_list, cal_cb_list):
    global matrix_selection_types, typenum
    """
    standard macbeth chart colour values
    """
    m_rgb = np.array([  # these are in RGB
        [116, 81, 67],    # dark skin
        [199, 147, 129],  # light skin
        [91, 122, 156],   # blue sky
        [90, 108, 64],    # foliage
        [130, 128, 176],  # blue flower
        [92, 190, 172],   # bluish green
        [224, 124, 47],   # orange
        [68, 91, 170],    # purplish blue
        [198, 82, 97],    # moderate red
        [94, 58, 106],    # purple
        [159, 189, 63],   # yellow green
        [230, 162, 39],   # orange yellow
        [35, 63, 147],    # blue
        [67, 149, 74],    # green
        [180, 49, 57],    # red
        [238, 198, 20],   # yellow
        [193, 84, 151],   # magenta
        [0, 136, 170],    # cyan (goes out of gamut)
        [245, 245, 243],  # white 9.5
        [200, 202, 202],  # neutral 8
        [161, 163, 163],  # neutral 6.5
        [121, 121, 122],  # neutral 5
        [82, 84, 86],     # neutral 3.5
        [49, 49, 51]      # black 2
    ])
    """
    convert reference colours from srgb to rgb
    """
    m_srgb = degamma(m_rgb)  # now in 16 bit color.

    # Produce array of LAB values for ideal color chart
    m_lab = [colors.RGB_to_LAB(color / 256) for color in m_srgb]

    """
    reorder reference values to match how patches are ordered
    """
    m_srgb = np.array([m_srgb[i::6] for i in range(6)]).reshape((24, 3))
    m_lab = np.array([m_lab[i::6] for i in range(6)]).reshape((24, 3))
    m_rgb = np.array([m_rgb[i::6] for i in range(6)]).reshape((24, 3))
    """
    reformat alsc correction tables or set colour_cals to None if alsc is
    deactivated
    """
    if cal_cr_list is None:
        colour_cals = None
    else:
        colour_cals = {}
        for cr, cb in zip(cal_cr_list, cal_cb_list):
            cr_tab = cr['table']
            cb_tab = cb['table']
            """
            normalise tables so min value is 1
            """
            cr_tab = cr_tab / np.min(cr_tab)
            cb_tab = cb_tab / np.min(cb_tab)
            colour_cals[cr['ct']] = [cr_tab, cb_tab]

    """
    for each image, perform awb and alsc corrections.
    Then calculate the colour correction matrix for that image, recording the
    ccm and the colour tempertaure.
    """
    ccm_tab = {}
    for Img in imgs:
        logger.info('Processing image: ' + Img.name)
        """
        get macbeth patches with alsc applied if alsc enabled.
        Note: if alsc is disabled then colour_cals will be set to None and no
        the function will simply return the macbeth patches
        """
        r, b, g = get_alsc_patches(Img, colour_cals, grey=False)
        # 256 values for each patch of sRGB values

        """
        do awb
        Note: awb is done by measuring the macbeth chart in the image, rather
        than from the awb calibration. This is done so the awb will be perfect
        and the ccm matrices will be more accurate.
        """
        r_greys, b_greys, g_greys = r[3::4], b[3::4], g[3::4]
        r_g = np.mean(r_greys / g_greys)
        b_g = np.mean(b_greys / g_greys)
        r = r / r_g
        b = b / b_g
        """
        normalise brightness wrt reference macbeth colours and then average
        each channel for each patch
        """
        gain = np.mean(m_srgb) / np.mean((r, g, b))
        logger.info(f'Gain with respect to standard colours: {gain:.3f}')
        r = np.mean(gain * r, axis=1)
        b = np.mean(gain * b, axis=1)
        g = np.mean(gain * g, axis=1)
        """
        calculate ccm matrix
        """
        # ==== All of below should in sRGB ===##
        sumde = 0
        ccm = do_ccm(r, g, b, m_srgb)
        # This is the initial guess that our optimisation code works with.
        original_ccm = ccm
        r1 = ccm[0]
        r2 = ccm[1]
        g1 = ccm[3]
        g2 = ccm[4]
        b1 = ccm[6]
        b2 = ccm[7]
        '''
        COLOR MATRIX LOOKS AS BELOW
        R1 R2 R3   Rval     Outr
        G1 G2 G3  *  Gval  =  G
        B1 B2 B3   Bval     B
        Will be optimising 6 elements and working out the third element using 1-r1-r2 = r3
        '''

        x0 = [r1, r2, g1, g2, b1, b2]
        '''
        We use our old CCM as the initial guess for the program to find the
        optimised matrix
        '''
        result = minimize(guess, x0, args=(r, g, b, m_lab), tol=0.01)
        '''
        This produces a color matrix which has the lowest delta E possible,
        based off the input data. Note it is impossible for this to reach
        zero since the input data is imperfect
        '''

        [r1, r2, g1, g2, b1, b2] = result.x
        # The new, optimised color correction matrix values
        # This is the optimised Color Matrix (preserving greys by summing rows up to 1)
        optimised_ccm = [r1, r2, (1 - r1 - r2), g1, g2, (1 - g1 - g2), b1, b2, (1 - b1 - b2)]

        logger.info(f'Optimized Matrix: {np.round(optimised_ccm, 4)}')
        logger.info(f'Old Matrix:       {np.round(ccm, 4)}')

        formatted_ccm = np.array(original_ccm).reshape((3, 3))

        '''
        below is a whole load of code that then applies the latest color
        matrix, and returns LAB values for color. This can then be used
        to calculate the final delta E
        '''
        optimised_ccm_rgb = []  # Original Color Corrected Matrix RGB / LAB
        optimised_ccm_lab = []

        formatted_optimised_ccm = np.array(optimised_ccm).reshape((3, 3))
        after_gamma_rgb = []
        after_gamma_lab = []

        for RGB in zip(r, g, b):
            ccm_applied_rgb = np.dot(formatted_ccm, (np.array(RGB) / 256))
            optimised_ccm_rgb.append(gamma(ccm_applied_rgb))
            optimised_ccm_lab.append(colors.RGB_to_LAB(ccm_applied_rgb))

            optimised_ccm_applied_rgb = np.dot(formatted_optimised_ccm, np.array(RGB) / 256)
            after_gamma_rgb.append(gamma(optimised_ccm_applied_rgb))
            after_gamma_lab.append(colors.RGB_to_LAB(optimised_ccm_applied_rgb))
        '''
        Gamma After RGB / LAB - not used in calculations, only used for visualisation
        We now want to spit out some data that shows
        how the optimisation has improved the color matrices
        '''
        logger.info("Here are the Improvements")

        # CALCULATE WORST CASE delta e
        old_worst_delta_e = 0
        before_average = transform_and_evaluate(formatted_ccm, r, g, b, m_lab)
        new_worst_delta_e = 0
        after_average = transform_and_evaluate(formatted_optimised_ccm, r, g, b, m_lab)
        for i in range(24):
            old_delta_e = deltae(optimised_ccm_lab[i], m_lab[i])  # Current Old Delta E
            new_delta_e = deltae(after_gamma_lab[i], m_lab[i])  # Current New Delta E
            if old_delta_e > old_worst_delta_e:
                old_worst_delta_e = old_delta_e
            if new_delta_e > new_worst_delta_e:
                new_worst_delta_e = new_delta_e

        logger.info(f'delta E optimized: average: {after_average:.2f}  max:{new_worst_delta_e:.2f}')
        logger.info(f'delta E old:       average: {before_average:.2f}  max:{old_worst_delta_e:.2f}')

        visualise_macbeth_chart(m_rgb, optimised_ccm_rgb, after_gamma_rgb, str(Img.color) + str(matrix_selection_types[typenum]))
        '''
        The program will also save some visualisations of improvements.
        Very pretty to look at. Top rectangle is ideal, Left square is
        before optimisation, right square is after.
        '''

        """
        if a ccm has already been calculated for that temperature then don't
        overwrite but save both. They will then be averaged later on
        """  # Now going to use optimised color matrix, optimised_ccm
        if Img.color in ccm_tab.keys():
            ccm_tab[Img.color].append(optimised_ccm)
        else:
            ccm_tab[Img.color] = [optimised_ccm]

    logger.info('Finished processing images')
    """
    average any ccms that share a colour temperature
    """
    for k, v in ccm_tab.items():
        tab = np.mean(v, axis=0)
        tab = np.where((10000 * tab) % 1 <= 0.05, tab + 0.00001, tab)
        tab = np.where((10000 * tab) % 1 >= 0.95, tab - 0.00001, tab)
        ccm_tab[k] = list(np.round(tab, 5))
        logger.info(f'Matrix calculated for colour temperature of {k} K')

    """
    return all ccms with respective colour temperature in the correct format,
    sorted by their colour temperature
    """
    sorted_ccms = sorted(ccm_tab.items(), key=lambda kv: kv[0])
    ccms = []
    for i in sorted_ccms:
        ccms.append({
            'ct': i[0],
            'ccm': i[1]
        })
    return ccms


def guess(x0, r, g, b, m_lab):       # provides a method of numerical feedback for the optimisation code
    [r1, r2, g1, g2, b1, b2] = x0
    ccm = np.array([r1, r2, (1 - r1 - r2),
                    g1, g2, (1 - g1 - g2),
                    b1, b2, (1 - b1 - b2)]).reshape((3, 3))  # format the matrix correctly
    return transform_and_evaluate(ccm, r, g, b, m_lab)


def transform_and_evaluate(ccm, r, g, b, m_lab):  # Transforms colors to LAB and applies the correction matrix
    # create list of matrix changed colors
    realrgb = []
    for RGB in zip(r, g, b):
        rgb_post_ccm = np.dot(ccm, np.array(RGB) / 256)  # This is RGB values after the color correction matrix has been applied
        realrgb.append(colors.RGB_to_LAB(rgb_post_ccm))
    # now compare that with m_lab and return numeric result, averaged for each patch
    return (sumde(realrgb, m_lab) / 24)  # returns an average result of delta E


def sumde(listA, listB):
    global typenum, test_patches
    sumde = 0
    maxde = 0
    patchde = []  # Create array of the delta E values for each patch. useful for optimisation of certain patches
    for listA_item, listB_item in zip(listA, listB):
        if maxde < (deltae(listA_item, listB_item)):
            maxde = deltae(listA_item, listB_item)
        patchde.append(deltae(listA_item, listB_item))
        sumde += deltae(listA_item, listB_item)
    '''
    The different options specified at the start allow for
    the maximum to be returned, average or specific patches
    '''
    if typenum == 0:
        return sumde
    if typenum == 1:
        return maxde
    if typenum == 2:
        output = sum([patchde[test_patch] for test_patch in test_patches])
        # Selects only certain patches and returns the output for them
        return output


"""
calculates the ccm for an individual image.
ccms are calculated in rgb space, and are fit by hand. Although it is a 3x3
matrix, each row must add up to 1 in order to conserve greyness, simplifying
calculation.
The initial CCM is calculated in RGB, and then optimised in LAB color space
This simplifies the initial calculation but then gets us the accuracy of
using LAB color space.
"""


def do_ccm(r, g, b, m_srgb):
    rb = r-b
    gb = g-b
    rb_2s = (rb * rb)
    rb_gbs = (rb * gb)
    gb_2s = (gb * gb)

    r_rbs = rb * (m_srgb[..., 0] - b)
    r_gbs = gb * (m_srgb[..., 0] - b)
    g_rbs = rb * (m_srgb[..., 1] - b)
    g_gbs = gb * (m_srgb[..., 1] - b)
    b_rbs = rb * (m_srgb[..., 2] - b)
    b_gbs = gb * (m_srgb[..., 2] - b)

    """
    Obtain least squares fit
    """
    rb_2 = np.sum(rb_2s)
    gb_2 = np.sum(gb_2s)
    rb_gb = np.sum(rb_gbs)
    r_rb = np.sum(r_rbs)
    r_gb = np.sum(r_gbs)
    g_rb = np.sum(g_rbs)
    g_gb = np.sum(g_gbs)
    b_rb = np.sum(b_rbs)
    b_gb = np.sum(b_gbs)

    det = rb_2 * gb_2 - rb_gb * rb_gb

    """
    Raise error if matrix is singular...
    This shouldn't really happen with real data but if it does just take new
    pictures and try again, not much else to be done unfortunately...
    """
    if det < 0.001:
        raise ArithmeticError

    r_a = (gb_2 * r_rb - rb_gb * r_gb) / det
    r_b = (rb_2 * r_gb - rb_gb * r_rb) / det
    """
    Last row can be calculated by knowing the sum must be 1
    """
    r_c = 1 - r_a - r_b

    g_a = (gb_2 * g_rb - rb_gb * g_gb) / det
    g_b = (rb_2 * g_gb - rb_gb * g_rb) / det
    g_c = 1 - g_a - g_b

    b_a = (gb_2 * b_rb - rb_gb * b_gb) / det
    b_b = (rb_2 * b_gb - rb_gb * b_rb) / det
    b_c = 1 - b_a - b_b

    """
    format ccm
    """
    ccm = [r_a, r_b, r_c, g_a, g_b, g_c, b_a, b_b, b_c]

    return ccm


def deltae(colorA, colorB):
    return ((colorA[0] - colorB[0]) ** 2 + (colorA[1] - colorB[1]) ** 2 + (colorA[2] - colorB[2]) ** 2) ** 0.5
    # return ((colorA[1]-colorB[1]) *  * 2 + (colorA[2]-colorB[2]) *  * 2) *  * 0.5
    # UNCOMMENT IF YOU WANT TO NEGLECT LUMINANCE FROM CALCULATION OF DELTA E
