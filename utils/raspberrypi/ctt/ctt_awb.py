# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_awb.py - camera tuning tool for AWB

from ctt_image_load import *
import matplotlib.pyplot as plt
from bisect import bisect_left
from scipy.optimize import fmin


"""
obtain piecewise linear approximation for colour curve
"""
def awb(Cam, cal_cr_list, cal_cb_list, plot):
    imgs = Cam.imgs
    """
    condense alsc calibration tables into one dictionary
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
    obtain data from greyscale macbeth patches
    """
    rb_raw = []
    rbs_hat = []
    for Img in imgs:
        Cam.log += '\nProcessing '+Img.name
        """
        get greyscale patches with alsc applied if alsc enabled.
        Note: if alsc is disabled then colour_cals will be set to None and the
        function will just return the greyscale patches
        """
        r_patchs, b_patchs, g_patchs = get_alsc_patches(Img, colour_cals)
        """
        calculate ratio of r, b to g
        """
        r_g = np.mean(r_patchs/g_patchs)
        b_g = np.mean(b_patchs/g_patchs)
        Cam.log += '\n       r : {:.4f}       b : {:.4f}'.format(r_g, b_g)
        """
        The curve tends to be better behaved in so-called hatspace.
        R, B, G represent the individual channels. The colour curve is plotted in
        r, b space, where:
            r = R/G
            b = B/G
        This will be referred to as dehatspace... (sorry)
        Hatspace is defined as:
            r_hat = R/(R+B+G)
            b_hat = B/(R+B+G)
        To convert from dehatspace to hastpace (hat operation):
            r_hat = r/(1+r+b)
            b_hat = b/(1+r+b)
        To convert from hatspace to dehatspace (dehat operation):
            r = r_hat/(1-r_hat-b_hat)
            b = b_hat/(1-r_hat-b_hat)
        Proof is left as an excercise to the reader...
        Throughout the code, r and b are sometimes referred to as r_g and b_g
        as a reminder that they are ratios
        """
        r_g_hat = r_g/(1+r_g+b_g)
        b_g_hat = b_g/(1+r_g+b_g)
        Cam.log += '\n   r_hat : {:.4f}   b_hat : {:.4f}'.format(r_g_hat, b_g_hat)
        rbs_hat.append((r_g_hat, b_g_hat, Img.col))
        rb_raw.append((r_g, b_g))
        Cam.log += '\n'

    Cam.log += '\nFinished processing images'
    """
    sort all lits simultaneously by r_hat
    """
    rbs_zip = list(zip(rbs_hat, rb_raw))
    rbs_zip.sort(key=lambda x: x[0][0])
    rbs_hat, rb_raw = list(zip(*rbs_zip))
    """
    unzip tuples ready for processing
    """
    rbs_hat = list(zip(*rbs_hat))
    rb_raw = list(zip(*rb_raw))
    """
    fit quadratic fit to r_g hat and b_g_hat
    """
    a, b, c = np.polyfit(rbs_hat[0], rbs_hat[1], 2)
    Cam.log += '\nFit quadratic curve in hatspace'
    """
    the algorithm now approximates the shortest distance from each point to the
    curve in dehatspace. Since the fit is done in hatspace, it is easier to
    find the actual shortest distance in hatspace and use the projection back
    into dehatspace as an overestimate.
    The distance will be used for two things:
        1) In the case that colour temperature does not strictly decrease with
        increasing r/g, the closest point to the line will be chosen out of an
        increasing pair of colours.

        2) To calculate transverse negative an dpositive, the maximum positive
        and negative distance from the line are chosen. This benefits from the
        overestimate as the transverse pos/neg are upper bound values.
    """
    """
    define fit function
    """
    def f(x):
        return a*x**2 + b*x + c
    """
    iterate over points (R, B are x and y coordinates of points) and calculate
    distance to line in dehatspace
    """
    dists = []
    for i, (R, B) in enumerate(zip(rbs_hat[0], rbs_hat[1])):
        """
        define function to minimise as square distance between datapoint and
        point on curve. Squaring is monotonic so minimising radius squared is
        equivalent to minimising radius
        """
        def f_min(x):
            y = f(x)
            return((x-R)**2+(y-B)**2)
        """
        perform optimisation with scipy.optmisie.fmin
        """
        x_hat = fmin(f_min, R, disp=0)[0]
        y_hat = f(x_hat)
        """
        dehat
        """
        x = x_hat/(1-x_hat-y_hat)
        y = y_hat/(1-x_hat-y_hat)
        rr = R/(1-R-B)
        bb = B/(1-R-B)
        """
        calculate euclidean distance in dehatspace
        """
        dist = ((x-rr)**2+(y-bb)**2)**0.5
        """
        return negative if point is below the fit curve
        """
        if (x+y) > (rr+bb):
            dist *= -1
        dists.append(dist)
    Cam.log += '\nFound closest point on fit line to each point in dehatspace'
    """
    calculate wiggle factors in awb. 10% added since this is an upper bound
    """
    transverse_neg = - np.min(dists) * 1.1
    transverse_pos = np.max(dists) * 1.1
    Cam.log += '\nTransverse pos : {:.5f}'.format(transverse_pos)
    Cam.log += '\nTransverse neg : {:.5f}'.format(transverse_neg)
    """
    set minimum transverse wiggles to 0.1 .
    Wiggle factors dictate how far off of the curve the algorithm searches. 0.1
    is a suitable minimum that gives better results for lighting conditions not
    within calibration dataset. Anything less will generalise poorly.
    """
    if transverse_pos < 0.01:
        transverse_pos = 0.01
        Cam.log += '\nForced transverse pos to 0.01'
    if transverse_neg < 0.01:
        transverse_neg = 0.01
        Cam.log += '\nForced transverse neg to 0.01'

    """
    generate new b_hat values at each r_hat according to fit
    """
    r_hat_fit = np.array(rbs_hat[0])
    b_hat_fit = a*r_hat_fit**2 + b*r_hat_fit + c
    """
    transform from hatspace to dehatspace
    """
    r_fit = r_hat_fit/(1-r_hat_fit-b_hat_fit)
    b_fit = b_hat_fit/(1-r_hat_fit-b_hat_fit)
    c_fit = np.round(rbs_hat[2], 0)
    """
    round to 4dp
    """
    r_fit = np.where((1000*r_fit) % 1 <= 0.05, r_fit+0.0001, r_fit)
    r_fit = np.where((1000*r_fit) % 1 >= 0.95, r_fit-0.0001, r_fit)
    b_fit = np.where((1000*b_fit) % 1 <= 0.05, b_fit+0.0001, b_fit)
    b_fit = np.where((1000*b_fit) % 1 >= 0.95, b_fit-0.0001, b_fit)
    r_fit = np.round(r_fit, 4)
    b_fit = np.round(b_fit, 4)
    """
    The following code ensures that colour temperature decreases with
    increasing r/g
    """
    """
    iterate backwards over list for easier indexing
    """
    i = len(c_fit) - 1
    while i > 0:
        if c_fit[i] > c_fit[i-1]:
            Cam.log += '\nColour temperature increase found\n'
            Cam.log += '{} K at r = {} to '.format(c_fit[i-1], r_fit[i-1])
            Cam.log += '{} K at r = {}'.format(c_fit[i], r_fit[i])
            """
            if colour temperature increases then discard point furthest from
            the transformed fit (dehatspace)
            """
            error_1 = abs(dists[i-1])
            error_2 = abs(dists[i])
            Cam.log += '\nDistances from fit:\n'
            Cam.log += '{} K : {:.5f} , '.format(c_fit[i], error_1)
            Cam.log += '{} K : {:.5f}'.format(c_fit[i-1], error_2)
            """
            find bad index
            note that in python false = 0 and true = 1
            """
            bad = i - (error_1 < error_2)
            Cam.log += '\nPoint at {} K deleted as '.format(c_fit[bad])
            Cam.log += 'it is furthest from fit'
            """
            delete bad point
            """
            r_fit = np.delete(r_fit, bad)
            b_fit = np.delete(b_fit, bad)
            c_fit = np.delete(c_fit, bad).astype(np.uint16)
        """
        note that if a point has been discarded then the length has decreased
        by one, meaning that decreasing the index by one will reassess the kept
        point against the next point. It is therefore possible, in theory, for
        two adjacent points to be discarded, although probably rare
        """
        i -= 1

    """
    return formatted ct curve, ordered by increasing colour temperature
    """
    ct_curve = list(np.array(list(zip(b_fit, r_fit, c_fit))).flatten())[::-1]
    Cam.log += '\nFinal CT curve:'
    for i in range(len(ct_curve)//3):
        j = 3*i
        Cam.log += '\n  ct: {}  '.format(ct_curve[j])
        Cam.log += '  r: {}  '.format(ct_curve[j+1])
        Cam.log += '  b: {}  '.format(ct_curve[j+2])

    """
    plotting code for debug
    """
    if plot:
        x = np.linspace(np.min(rbs_hat[0]), np.max(rbs_hat[0]), 100)
        y = a*x**2 + b*x + c
        plt.subplot(2, 1, 1)
        plt.title('hatspace')
        plt.plot(rbs_hat[0], rbs_hat[1], ls='--', color='blue')
        plt.plot(x, y, color='green', ls='-')
        plt.scatter(rbs_hat[0], rbs_hat[1], color='red')
        for i, ct in enumerate(rbs_hat[2]):
            plt.annotate(str(ct), (rbs_hat[0][i], rbs_hat[1][i]))
        plt.xlabel('$\\hat{r}$')
        plt.ylabel('$\\hat{b}$')
        """
        optional set axes equal to shortest distance so line really does
        looks perpendicular and everybody is happy
        """
        # ax = plt.gca()
        # ax.set_aspect('equal')
        plt.grid()
        plt.subplot(2, 1, 2)
        plt.title('dehatspace - indoors?')
        plt.plot(r_fit, b_fit, color='blue')
        plt.scatter(rb_raw[0], rb_raw[1], color='green')
        plt.scatter(r_fit, b_fit, color='red')
        for i, ct in enumerate(c_fit):
            plt.annotate(str(ct), (r_fit[i], b_fit[i]))
        plt.xlabel('$r$')
        plt.ylabel('$b$')
        """
        optional set axes equal to shortest distance so line really does
        looks perpendicular and everybody is happy
        """
        # ax = plt.gca()
        # ax.set_aspect('equal')
        plt.subplots_adjust(hspace=0.5)
        plt.grid()
        plt.show()
    """
    end of plotting code
    """
    return(ct_curve, np.round(transverse_pos, 5), np.round(transverse_neg, 5))


"""
obtain greyscale patches and perform alsc colour correction
"""
def get_alsc_patches(Img, colour_cals, grey=True):
    """
    get patch centre coordinates, image colour and the actual
    patches for each channel, remembering to subtract blacklevel
    If grey then only greyscale patches considered
    """
    if grey:
        cen_coords = Img.cen_coords[3::4]
        col = Img.col
        patches = [np.array(Img.patches[i]) for i in Img.order]
        r_patchs = patches[0][3::4] - Img.blacklevel_16
        b_patchs = patches[3][3::4] - Img.blacklevel_16
        """
        note two green channels are averages
        """
        g_patchs = (patches[1][3::4]+patches[2][3::4])/2 - Img.blacklevel_16
    else:
        cen_coords = Img.cen_coords
        col = Img.col
        patches = [np.array(Img.patches[i]) for i in Img.order]
        r_patchs = patches[0] - Img.blacklevel_16
        b_patchs = patches[3] - Img.blacklevel_16
        g_patchs = (patches[1]+patches[2])/2 - Img.blacklevel_16

    if colour_cals is None:
        return r_patchs, b_patchs, g_patchs
    """
    find where image colour fits in alsc colour calibration tables
    """
    cts = list(colour_cals.keys())
    pos = bisect_left(cts, col)
    """
    if img colour is below minimum or above maximum alsc calibration colour, simply
    pick extreme closest to img colour
    """
    if pos % len(cts) == 0:
        """
        this works because -0 = 0 = first and -1 = last index
        """
        col_tabs = np.array(colour_cals[cts[-pos//len(cts)]])
        """
    else, perform linear interpolation between existing alsc colour
    calibration tables
    """
    else:
        bef = cts[pos-1]
        aft = cts[pos]
        da = col-bef
        db = aft-col
        bef_tabs = np.array(colour_cals[bef])
        aft_tabs = np.array(colour_cals[aft])
        col_tabs = (bef_tabs*db + aft_tabs*da)/(da+db)
    col_tabs = np.reshape(col_tabs, (2, 12, 16))
    """
    calculate dx, dy used to calculate alsc table
    """
    w, h = Img.w/2, Img.h/2
    dx, dy = int(-(-(w-1)//16)), int(-(-(h-1)//12))
    """
    make list of pairs of gains for each patch by selecting the correct value
    in alsc colour calibration table
    """
    patch_gains = []
    for cen in cen_coords:
        x, y = cen[0]//dx, cen[1]//dy
        # We could probably do with some better spatial interpolation here?
        col_gains = (col_tabs[0][y][x], col_tabs[1][y][x])
        patch_gains.append(col_gains)

    """
    multiply the r and b channels in each patch by the respective gain, finally
    performing the alsc colour correction
    """
    for i, gains in enumerate(patch_gains):
        r_patchs[i] = r_patchs[i] * gains[0]
        b_patchs[i] = b_patchs[i] * gains[1]

    """
    return greyscale patches, g channel and correct r, b channels
    """
    return r_patchs, b_patchs, g_patchs
