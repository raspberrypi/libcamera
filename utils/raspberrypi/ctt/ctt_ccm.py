# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_ccm.py - camera tuning tool for CCM (colour correction matrix)

from ctt_image_load import *
from ctt_awb import get_alsc_patches


"""
takes 8-bit macbeth chart values, degammas and returns 16 bit
"""
def degamma(x):
    x = x / ((2**8)-1)
    x = np.where(x < 0.04045, x/12.92, ((x+0.055)/1.055)**2.4)
    x = x * ((2**16)-1)
    return x


"""
FInds colour correction matrices for list of images
"""
def ccm(Cam, cal_cr_list, cal_cb_list):
    imgs = Cam.imgs
    """
    standard macbeth chart colour values
    """
    m_rgb = np.array([  # these are in sRGB
        [116, 81, 67],    # dark skin
        [199, 147, 129],  # light skin
        [91, 122, 156],   # blue sky
        [90, 108, 64],    # foliage
        [130, 128, 176],  # blue flower
        [92, 190, 172],   # bluish green
        [224, 124, 47],   # orange
        [68, 91, 170],     # purplish blue
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
    m_srgb = degamma(m_rgb)
    """
    reorder reference values to match how patches are ordered
    """
    m_srgb = np.array([m_srgb[i::6] for i in range(6)]).reshape((24, 3))

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
            cr_tab = cr_tab/np.min(cr_tab)
            cb_tab = cb_tab/np.min(cb_tab)
            colour_cals[cr['ct']] = [cr_tab, cb_tab]

    """
    for each image, perform awb and alsc corrections.
    Then calculate the colour correction matrix for that image, recording the
    ccm and the colour tempertaure.
    """
    ccm_tab = {}
    for Img in imgs:
        Cam.log += '\nProcessing image: ' + Img.name
        """
        get macbeth patches with alsc applied if alsc enabled.
        Note: if alsc is disabled then colour_cals will be set to None and no
        the function will simply return the macbeth patches
        """
        r, b, g = get_alsc_patches(Img, colour_cals, grey=False)
        """
        do awb
        Note: awb is done by measuring the macbeth chart in the image, rather
        than from the awb calibration. This is done so the awb will be perfect
        and the ccm matrices will be more accurate.
        """
        r_greys, b_greys, g_greys = r[3::4], b[3::4], g[3::4]
        r_g = np.mean(r_greys/g_greys)
        b_g = np.mean(b_greys/g_greys)
        r = r / r_g
        b = b / b_g

        """
        normalise brightness wrt reference macbeth colours and then average
        each channel for each patch
        """
        gain = np.mean(m_srgb)/np.mean((r, g, b))
        Cam.log += '\nGain with respect to standard colours: {:.3f}'.format(gain)
        r = np.mean(gain*r, axis=1)
        b = np.mean(gain*b, axis=1)
        g = np.mean(gain*g, axis=1)

        """
        calculate ccm matrix
        """
        ccm = do_ccm(r, g, b, m_srgb)

        """
        if a ccm has already been calculated for that temperature then don't
        overwrite but save both. They will then be averaged later on
        """
        if Img.col in ccm_tab.keys():
            ccm_tab[Img.col].append(ccm)
        else:
            ccm_tab[Img.col] = [ccm]
        Cam.log += '\n'

    Cam.log += '\nFinished processing images'
    """
    average any ccms that share a colour temperature
    """
    for k, v in ccm_tab.items():
        tab = np.mean(v, axis=0)
        tab = np.where((10000*tab) % 1 <= 0.05, tab+0.00001, tab)
        tab = np.where((10000*tab) % 1 >= 0.95, tab-0.00001, tab)
        ccm_tab[k] = list(np.round(tab, 5))
        Cam.log += '\nMatrix calculated for colour temperature of {} K'.format(k)

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


"""
calculates the ccm for an individual image.
ccms are calculate in rgb space, and are fit by hand. Although it is a 3x3
matrix, each row must add up to 1 in order to conserve greyness, simplifying
calculation.
Should you want to fit them in another space (e.g. LAB) we wish you the best of
luck and send us the code when you are done! :-)
"""
def do_ccm(r, g, b, m_srgb):
    rb = r-b
    gb = g-b
    rb_2s = (rb*rb)
    rb_gbs = (rb*gb)
    gb_2s = (gb*gb)

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

    det = rb_2*gb_2 - rb_gb*rb_gb

    """
    Raise error if matrix is singular...
    This shouldn't really happen with real data but if it does just take new
    pictures and try again, not much else to be done unfortunately...
    """
    if det < 0.001:
        raise ArithmeticError

    r_a = (gb_2*r_rb - rb_gb*r_gb)/det
    r_b = (rb_2*r_gb - rb_gb*r_rb)/det
    """
    Last row can be calculated by knowing the sum must be 1
    """
    r_c = 1 - r_a - r_b

    g_a = (gb_2*g_rb - rb_gb*g_gb)/det
    g_b = (rb_2*g_gb - rb_gb*g_rb)/det
    g_c = 1 - g_a - g_b

    b_a = (gb_2*b_rb - rb_gb*b_gb)/det
    b_b = (rb_2*b_gb - rb_gb*b_rb)/det
    b_c = 1 - b_a - b_b

    """
    format ccm
    """
    ccm = [r_a, r_b, r_c, g_a, g_b, g_c, b_a, b_b, b_c]

    return ccm
