# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# camera tuning tool for ALSC (auto lens shading correction)

from ctt_image_load import *
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D


"""
preform alsc calibration on a set of images
"""
def alsc_all(Cam, do_alsc_colour, plot, grid_size=(16, 12), max_gain=8.0):
    imgs_alsc = Cam.imgs_alsc
    grid_w, grid_h = grid_size
    """
    create list of colour temperatures and associated calibration tables
    """
    list_col = []
    list_cr = []
    list_cb = []
    list_cg = []
    for Img in imgs_alsc:
        col, cr, cb, cg, size = alsc(Cam, Img, do_alsc_colour, plot, grid_size=grid_size, max_gain=max_gain)
        list_col.append(col)
        list_cr.append(cr)
        list_cb.append(cb)
        list_cg.append(cg)
        Cam.log += '\n'
    Cam.log += '\nFinished processing images'
    w, h, dx, dy = size
    Cam.log += '\nChannel dimensions: w = {}  h = {}'.format(int(w), int(h))
    Cam.log += '\n16x12 grid rectangle size: w = {} h = {}'.format(dx, dy)

    """
    convert to numpy array for data manipulation
    """
    list_col = np.array(list_col)
    list_cr = np.array(list_cr)
    list_cb = np.array(list_cb)
    list_cg = np.array(list_cg)

    cal_cr_list = []
    cal_cb_list = []

    """
    only do colour calculations if required
    """
    if do_alsc_colour:
        Cam.log += '\nALSC colour tables'
        for ct in sorted(set(list_col)):
            Cam.log += '\nColour temperature: {} K'.format(ct)
            """
            average tables for the same colour temperature
            """
            indices = np.where(list_col == ct)
            ct = int(ct)
            t_r = np.mean(list_cr[indices], axis=0)
            t_b = np.mean(list_cb[indices], axis=0)
            """
            force numbers to be stored to 3dp.... :(
            """
            t_r = np.where((100*t_r) % 1 <= 0.05, t_r+0.001, t_r)
            t_b = np.where((100*t_b) % 1 <= 0.05, t_b+0.001, t_b)
            t_r = np.where((100*t_r) % 1 >= 0.95, t_r-0.001, t_r)
            t_b = np.where((100*t_b) % 1 >= 0.95, t_b-0.001, t_b)
            t_r = np.round(t_r, 3)
            t_b = np.round(t_b, 3)
            r_corners = (t_r[0], t_r[grid_w - 1], t_r[-1], t_r[-grid_w])
            b_corners = (t_b[0], t_b[grid_w - 1], t_b[-1], t_b[-grid_w])
            middle_pos = (grid_h // 2 - 1) * grid_w + grid_w - 1
            r_cen = t_r[middle_pos]+t_r[middle_pos + 1]+t_r[middle_pos + grid_w]+t_r[middle_pos + grid_w + 1]
            r_cen = round(r_cen/4, 3)
            b_cen = t_b[middle_pos]+t_b[middle_pos + 1]+t_b[middle_pos + grid_w]+t_b[middle_pos + grid_w + 1]
            b_cen = round(b_cen/4, 3)
            Cam.log += '\nRed table corners: {}'.format(r_corners)
            Cam.log += '\nRed table centre: {}'.format(r_cen)
            Cam.log += '\nBlue table corners: {}'.format(b_corners)
            Cam.log += '\nBlue table centre: {}'.format(b_cen)
            cr_dict = {
                'ct': ct,
                'table': list(t_r)
            }
            cb_dict = {
                'ct': ct,
                'table': list(t_b)
            }
            cal_cr_list.append(cr_dict)
            cal_cb_list.append(cb_dict)
            Cam.log += '\n'
    else:
        cal_cr_list, cal_cb_list = None, None

    """
    average all values for luminance shading and return one table for all temperatures
    """
    lum_lut = np.mean(list_cg, axis=0)
    lum_lut = np.where((100*lum_lut) % 1 <= 0.05, lum_lut+0.001, lum_lut)
    lum_lut = np.where((100*lum_lut) % 1 >= 0.95, lum_lut-0.001, lum_lut)
    lum_lut = list(np.round(lum_lut, 3))

    """
    calculate average corner for lsc gain calculation further on
    """
    corners = (lum_lut[0], lum_lut[15], lum_lut[-1], lum_lut[-16])
    Cam.log += '\nLuminance table corners: {}'.format(corners)
    l_cen = lum_lut[5*16+7]+lum_lut[5*16+8]+lum_lut[6*16+7]+lum_lut[6*16+8]
    l_cen = round(l_cen/4, 3)
    Cam.log += '\nLuminance table centre: {}'.format(l_cen)
    av_corn = np.sum(corners)/4

    return cal_cr_list, cal_cb_list, lum_lut, av_corn


"""
calculate g/r and g/b for 32x32 points arranged in a grid for a single image
"""
def alsc(Cam, Img, do_alsc_colour, plot=False, grid_size=(16, 12), max_gain=8.0):
    Cam.log += '\nProcessing image: ' + Img.name
    grid_w, grid_h = grid_size
    """
    get channel in correct order
    """
    channels = [Img.channels[i] for i in Img.order]
    """
    calculate size of single rectangle.
    The divisions here must ensure the final row/column of cells has a non-zero number of
    pixels.
    """
    w, h = Img.w/2, Img.h/2
    dx, dy = int((w - 1) // (grid_w - 1)), int((h - 1) // (grid_h - 1))

    """
    average the green channels into one
    """
    av_ch_g = np.mean((channels[1:3]), axis=0)
    if do_alsc_colour:
        """
        obtain grid_w x grid_h grid of intensities for each channel and subtract black level
        """
        g = get_grid(av_ch_g, dx, dy, grid_size) - Img.blacklevel_16
        r = get_grid(channels[0], dx, dy, grid_size) - Img.blacklevel_16
        b = get_grid(channels[3], dx, dy, grid_size) - Img.blacklevel_16
        """
        calculate ratios as 32 bit in order to be supported by medianBlur function
        """
        cr = np.reshape(g/r, (grid_h, grid_w)).astype('float32')
        cb = np.reshape(g/b, (grid_h, grid_w)).astype('float32')
        cg = np.reshape(1/g, (grid_h, grid_w)).astype('float32')
        """
        median blur to remove peaks and save as float 64
        """
        cr = cv2.medianBlur(cr, 3).astype('float64')
        cr = cr/np.min(cr)  # gain tables are easier for humans to read if the minimum is 1.0
        cb = cv2.medianBlur(cb, 3).astype('float64')
        cb = cb/np.min(cb)
        cg = cv2.medianBlur(cg, 3).astype('float64')
        cg = cg/np.min(cg)
        cg = [min(v, max_gain) for v in cg.flatten()]  # never exceed the max luminance gain

        """
        debugging code showing 2D surface plot of vignetting. Quite useful for
        for sanity check
        """
        if plot:
            hf = plt.figure(figsize=(8, 8))
            ha = hf.add_subplot(311, projection='3d')
            """
            note Y is plotted as -Y so plot has same axes as image
            """
            X, Y = np.meshgrid(range(grid_w), range(grid_h))
            ha.plot_surface(X, -Y, cr, cmap=cm.coolwarm, linewidth=0)
            ha.set_title('ALSC Plot\nImg: {}\n\ncr'.format(Img.str))
            hb = hf.add_subplot(312, projection='3d')
            hb.plot_surface(X, -Y, cb, cmap=cm.coolwarm, linewidth=0)
            hb.set_title('cb')
            hc = hf.add_subplot(313, projection='3d')
            hc.plot_surface(X, -Y, cg, cmap=cm.coolwarm, linewidth=0)
            hc.set_title('g')
            # print(Img.str)
            plt.show()

        return Img.col, cr.flatten(), cb.flatten(), cg, (w, h, dx, dy)

    else:
        """
        only perform calculations for luminance shading
        """
        g = get_grid(av_ch_g, dx, dy, grid_size) - Img.blacklevel_16
        cg = np.reshape(1/g, (grid_h, grid_w)).astype('float32')
        cg = cv2.medianBlur(cg, 3).astype('float64')
        cg = cg/np.min(cg)
        cg = [min(v, max_gain) for v in cg.flatten()]  # never exceed the max luminance gain

        if plot:
            hf = plt.figure(figssize=(8, 8))
            ha = hf.add_subplot(1, 1, 1, projection='3d')
            X, Y = np.meashgrid(range(grid_w), range(grid_h))
            ha.plot_surface(X, -Y, cg, cmap=cm.coolwarm, linewidth=0)
            ha.set_title('ALSC Plot (Luminance only!)\nImg: {}\n\ncg').format(Img.str)
            plt.show()

        return Img.col, None, None, cg.flatten(), (w, h, dx, dy)


"""
Compresses channel down to a grid of the requested size
"""
def get_grid(chan, dx, dy, grid_size):
    grid_w, grid_h = grid_size
    grid = []
    """
    since left and bottom border will not necessarily have rectangles of
    dimension dx x dy, the 32nd iteration has to be handled separately.
    """
    for i in range(grid_h - 1):
        for j in range(grid_w - 1):
            grid.append(np.mean(chan[dy*i:dy*(1+i), dx*j:dx*(1+j)]))
        grid.append(np.mean(chan[dy*i:dy*(1+i), (grid_w - 1)*dx:]))
    for j in range(grid_w - 1):
        grid.append(np.mean(chan[(grid_h - 1)*dy:, dx*j:dx*(1+j)]))
    grid.append(np.mean(chan[(grid_h - 1)*dy:, (grid_w - 1)*dx:]))
    """
    return as np.array, ready for further manipulation
    """
    return np.array(grid)


"""
obtains sigmas for red and blue, effectively a measure of the 'error'
"""
def get_sigma(Cam, cal_cr_list, cal_cb_list, grid_size):
    Cam.log += '\nCalculating sigmas'
    """
    provided colour alsc tables were generated for two different colour
    temperatures sigma is calculated by comparing two calibration temperatures
    adjacent in colour space
    """
    """
    create list of colour temperatures
    """
    cts = [cal['ct'] for cal in cal_cr_list]
    # print(cts)
    """
    calculate sigmas for each adjacent cts and return worst one
    """
    sigma_rs = []
    sigma_bs = []
    for i in range(len(cts)-1):
        sigma_rs.append(calc_sigma(cal_cr_list[i]['table'], cal_cr_list[i+1]['table'], grid_size))
        sigma_bs.append(calc_sigma(cal_cb_list[i]['table'], cal_cb_list[i+1]['table'], grid_size))
        Cam.log += '\nColour temperature interval {} - {} K'.format(cts[i], cts[i+1])
        Cam.log += '\nSigma red: {}'.format(sigma_rs[-1])
        Cam.log += '\nSigma blue: {}'.format(sigma_bs[-1])

    """
    return maximum sigmas, not necessarily from the same colour temperature
    interval
    """
    sigma_r = max(sigma_rs) if sigma_rs else 0.005
    sigma_b = max(sigma_bs) if sigma_bs else 0.005
    Cam.log += '\nMaximum sigmas: Red = {} Blue = {}'.format(sigma_r, sigma_b)

    # print(sigma_rs, sigma_bs)
    # print(sigma_r, sigma_b)
    return sigma_r, sigma_b


"""
calculate sigma from two adjacent gain tables
"""
def calc_sigma(g1, g2, grid_size):
    grid_w, grid_h = grid_size
    """
    reshape into 16x12 matrix
    """
    g1 = np.reshape(g1, (grid_h, grid_w))
    g2 = np.reshape(g2, (grid_h, grid_w))
    """
    apply gains to gain table
    """
    gg = g1/g2
    if np.mean(gg) < 1:
        gg = 1/gg
    """
    for each internal patch, compute average difference between it and its 4
    neighbours, then append to list
    """
    diffs = []
    for i in range(grid_h - 2):
        for j in range(grid_w - 2):
            """
            note indexing is incremented by 1 since all patches on borders are
            not counted
            """
            diff = np.abs(gg[i+1][j+1]-gg[i][j+1])
            diff += np.abs(gg[i+1][j+1]-gg[i+2][j+1])
            diff += np.abs(gg[i+1][j+1]-gg[i+1][j])
            diff += np.abs(gg[i+1][j+1]-gg[i+1][j+2])
            diffs.append(diff/4)

    """
    return mean difference
    """
    mean_diff = np.mean(diffs)
    return(np.round(mean_diff, 5))
