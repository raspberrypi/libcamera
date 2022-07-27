# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_noise.py - camera tuning tool noise calibration

from ctt_image_load import *
import matplotlib.pyplot as plt


"""
Find noise standard deviation and fit to model:

    noise std = a + b*sqrt(pixel mean)
"""
def noise(Cam, Img, plot):
    Cam.log += '\nProcessing image: {}'.format(Img.name)
    stds = []
    means = []
    """
    iterate through macbeth square patches
    """
    for ch_patches in Img.patches:
        for patch in ch_patches:
            """
            renormalise patch
            """
            patch = np.array(patch)
            patch = (patch-Img.blacklevel_16)/Img.againQ8_norm
            std = np.std(patch)
            mean = np.mean(patch)
            stds.append(std)
            means.append(mean)

    """
    clean data and ensure all means are above 0
    """
    stds = np.array(stds)
    means = np.array(means)
    means = np.clip(np.array(means), 0, None)
    sq_means = np.sqrt(means)

    """
    least squares fit model
    """
    fit = np.polyfit(sq_means, stds, 1)
    Cam.log += '\nBlack level = {}'.format(Img.blacklevel_16)
    Cam.log += '\nNoise profile: offset = {}'.format(int(fit[1]))
    Cam.log += ' slope = {:.3f}'.format(fit[0])
    """
    remove any values further than std from the fit

    anomalies most likely caused by:
    > ucharacteristically noisy white patch
    > saturation in the white patch
    """
    fit_score = np.abs(stds - fit[0]*sq_means - fit[1])
    fit_std = np.std(stds)
    fit_score_norm = fit_score - fit_std
    anom_ind = np.where(fit_score_norm > 1)
    fit_score_norm.sort()
    sq_means_clean = np.delete(sq_means, anom_ind)
    stds_clean = np.delete(stds, anom_ind)
    removed = len(stds) - len(stds_clean)
    if removed != 0:
        Cam.log += '\nIdentified and removed {} anomalies.'.format(removed)
        Cam.log += '\nRecalculating fit'
        """
        recalculate fit with outliers removed
        """
        fit = np.polyfit(sq_means_clean, stds_clean, 1)
        Cam.log += '\nNoise profile: offset = {}'.format(int(fit[1]))
        Cam.log += ' slope = {:.3f}'.format(fit[0])

    """
    if fit const is < 0 then force through 0 by
    dividing by sq_means and fitting poly order 0
    """
    corrected = 0
    if fit[1] < 0:
        corrected = 1
        ones = np.ones(len(means))
        y_data = stds/sq_means
        fit2 = np.polyfit(ones, y_data, 0)
        Cam.log += '\nOffset below zero. Fit recalculated with zero offset'
        Cam.log += '\nNoise profile: offset = 0'
        Cam.log += ' slope = {:.3f}'.format(fit2[0])
        # print('new fit')
        # print(fit2)

    """
    plot fit for debug
    """
    if plot:
        x = np.arange(sq_means.max()//0.88)
        fit_plot = x*fit[0] + fit[1]
        plt.scatter(sq_means, stds, label='data', color='blue')
        plt.scatter(sq_means[anom_ind], stds[anom_ind], color='orange', label='anomalies')
        plt.plot(x, fit_plot, label='fit', color='red', ls=':')
        if fit[1] < 0:
            fit_plot_2 = x*fit2[0]
            plt.plot(x, fit_plot_2, label='fit 0 intercept', color='green', ls='--')
        plt.plot(0, 0)
        plt.title('Noise Plot\nImg: {}'.format(Img.str))
        plt.legend(loc='upper left')
        plt.xlabel('Sqrt Pixel Value')
        plt.ylabel('Noise Standard Deviation')
        plt.grid()
        plt.show()
    """
    End of plotting code
    """

    """
    format output to include forced 0 constant
    """
    Cam.log += '\n'
    if corrected:
        fit = [fit2[0], 0]
        return fit

    else:
        return fit
