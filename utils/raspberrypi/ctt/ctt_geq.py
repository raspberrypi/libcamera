# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_geq.py - camera tuning tool for GEQ (green equalisation)

from ctt_tools import *
import matplotlib.pyplot as plt
import scipy.optimize as optimize


"""
Uses green differences in macbeth patches to fit green equalisation threshold
model. Ideally, all macbeth chart centres would fall below the threshold as
these should be corrected by geq.
"""
def geq_fit(Cam, plot):
    imgs = Cam.imgs
    """
    green equalisation to mitigate mazing.
    Fits geq model by looking at difference
    between greens in macbeth patches
    """
    geqs = np.array([geq(Cam, Img)*Img.againQ8_norm for Img in imgs])
    Cam.log += '\nProcessed all images'
    geqs = geqs.reshape((-1, 2))
    """
    data is sorted by green difference and top half is selected since higher
    green difference data define the decision boundary.
    """
    geqs = np.array(sorted(geqs, key=lambda r: np.abs((r[1]-r[0])/r[0])))

    length = len(geqs)
    g0 = geqs[length//2:, 0]
    g1 = geqs[length//2:, 1]
    gdiff = np.abs(g0-g1)
    """
    find linear fit by minimising asymmetric least square errors
    in order to cover most of the macbeth images.
    the philosophy here is that every macbeth patch should fall within the
    threshold, hence the upper bound approach
    """
    def f(params):
        m, c = params
        a = gdiff - (m*g0+c)
        """
        asymmetric square error returns:
            1.95 * a**2 if a is positive
            0.05 * a**2 if a is negative
        """
        return(np.sum(a**2+0.95*np.abs(a)*a))

    initial_guess = [0.01, 500]
    """
    Nelder-Mead is usually not the most desirable optimisation method
    but has been chosen here due to its robustness to undifferentiability
    (is that a word?)
    """
    result = optimize.minimize(f, initial_guess, method='Nelder-Mead')
    """
    need to check if the fit worked correectly
    """
    if result.success:
        slope, offset = result.x
        Cam.log += '\nFit result: slope = {:.5f} '.format(slope)
        Cam.log += 'offset = {}'.format(int(offset))
        """
        optional plotting code
        """
        if plot:
            x = np.linspace(max(g0)*1.1, 100)
            y = slope*x + offset
            plt.title('GEQ Asymmetric \'Upper Bound\' Fit')
            plt.plot(x, y, color='red', ls='--', label='fit')
            plt.scatter(g0, gdiff, color='b', label='data')
            plt.ylabel('Difference in green channels')
            plt.xlabel('Green value')

        """
        This upper bound asymmetric gives correct order of magnitude values.
        The pipeline approximates a 1st derivative of a gaussian with some
        linear piecewise functions, introducing arbitrary cutoffs. For
        pessimistic geq, the model parameters have been increased by a
        scaling factor/constant.

        Feel free to tune these or edit the json files directly if you
        belive there are still mazing effects left (threshold too low) or if you
        think it is being overcorrected (threshold too high).
        We have gone for a one size fits most approach that will produce
        acceptable results in most applications.
        """
        slope *= 1.5
        offset += 201
        Cam.log += '\nFit after correction factors: slope = {:.5f}'.format(slope)
        Cam.log += ' offset = {}'.format(int(offset))
        """
        clamp offset at 0 due to pipeline considerations
        """
        if offset < 0:
            Cam.log += '\nOffset raised to 0'
            offset = 0
        """
        optional plotting code
        """
        if plot:
            y2 = slope*x + offset
            plt.plot(x, y2, color='green', ls='--', label='scaled fit')
            plt.grid()
            plt.legend()
            plt.show()

        """
    the case where for some reason the fit didn't work correctly

    Transpose data and then least squares linear fit. Transposing data
    makes it robust to many patches where green difference is the same
    since they only contribute to one error minimisation, instead of dragging
    the entire linear fit down.
    """

    else:
        print('\nError! Couldn\'t fit asymmetric lest squares')
        print(result.message)
        Cam.log += '\nWARNING: Asymmetric least squares fit failed! '
        Cam.log += 'Standard fit used could possibly lead to worse results'
        fit = np.polyfit(gdiff, g0, 1)
        offset, slope = -fit[1]/fit[0], 1/fit[0]
        Cam.log += '\nFit result: slope = {:.5f} '.format(slope)
        Cam.log += 'offset = {}'.format(int(offset))
        """
        optional plotting code
        """
        if plot:
            x = np.linspace(max(g0)*1.1, 100)
            y = slope*x + offset
            plt.title('GEQ Linear Fit')
            plt.plot(x, y, color='red', ls='--', label='fit')
            plt.scatter(g0, gdiff, color='b', label='data')
            plt.ylabel('Difference in green channels')
            plt.xlabel('Green value')
        """
        Scaling factors (see previous justification)
        The model here will not be an upper bound so scaling factors have
        been increased.
        This method of deriving geq model parameters is extremely arbitrary
        and undesirable.
        """
        slope *= 2.5
        offset += 301
        Cam.log += '\nFit after correction factors: slope = {:.5f}'.format(slope)
        Cam.log += ' offset = {}'.format(int(offset))

        if offset < 0:
            Cam.log += '\nOffset raised to 0'
            offset = 0

        """
        optional plotting code
        """
        if plot:
            y2 = slope*x + offset
            plt.plot(x, y2, color='green', ls='--', label='scaled fit')
            plt.legend()
            plt.grid()
            plt.show()

    return round(slope, 5), int(offset)


""""
Return green channels of macbeth patches
returns g0, g1 where
> g0 is green next to red
> g1 is green next to blue
"""
def geq(Cam, Img):
    Cam.log += '\nProcessing image {}'.format(Img.name)
    patches = [Img.patches[i] for i in Img.order][1:3]
    g_patches = np.array([(np.mean(patches[0][i]), np.mean(patches[1][i])) for i in range(24)])
    Cam.log += '\n'
    return(g_patches)
