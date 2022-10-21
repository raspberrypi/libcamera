# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# raspberrypi.py - ALSC module for tuning Raspberry Pi

from .lsc import LSC

import libtuning as lt
import libtuning.utils as utils

from numbers import Number
import numpy as np


class ALSCRaspberryPi(LSC):
    # Override the type name so that the parser can match the entry in the
    # config file.
    type = 'alsc'
    hr_name = 'ALSC (Raspberry Pi)'
    out_name = 'rpi.alsc'
    compatible = ['raspberrypi']

    def __init__(self, *,
                 do_color: lt.Param,
                 luminance_strength: lt.Param,
                 **kwargs):
        super().__init__(**kwargs)

        self.do_color = do_color
        self.luminance_strength = luminance_strength

        self.output_range = (0, 3.999)

    def validate_config(self, config: dict) -> bool:
        if self not in config:
            utils.eprint(f'{self.type} not in config')
            return False

        valid = True

        conf = config[self]

        lum_key = self.luminance_strength.name
        color_key = self.do_color.name

        if lum_key not in conf and self.luminance_strength.required:
            utils.eprint(f'{lum_key} is not in config')
            valid = False

        if lum_key in conf and (conf[lum_key] < 0 or conf[lum_key] > 1):
            utils.eprint(f'Warning: {lum_key} is not in range [0, 1]; defaulting to 0.5')

        if color_key not in conf and self.do_color.required:
            utils.eprint(f'{color_key} is not in config')
            valid = False

        return valid

    # @return Image color temperature, flattened array of red calibration table
    #         (containing {sector size} elements), flattened array of blue
    #         calibration table, flattened array of green calibration
    #         table

    def _do_single_alsc(self, image: lt.Image, do_alsc_colour: bool):
        average_green = np.mean((image.channels[lt.Color.GR:lt.Color.GB + 1]), axis=0)

        cg, g = self._lsc_single_channel(average_green, image)

        if not do_alsc_colour:
            return image.color, None, None, cg.flatten()

        cr, _ = self._lsc_single_channel(image.channels[lt.Color.R], image, g)
        cb, _ = self._lsc_single_channel(image.channels[lt.Color.B], image, g)

        # \todo implement debug

        return image.color, cr.flatten(), cb.flatten(), cg.flatten()

    # @return Red shading table, Blue shading table, Green shading table,
    #         number of images processed

    def _do_all_alsc(self, images: list, do_alsc_colour: bool, general_conf: dict) -> (list, list, list, Number, int):
        # List of colour temperatures
        list_col = []
        # Associated calibration tables
        list_cr = []
        list_cb = []
        list_cg = []
        count = 0
        for image in self._enumerate_lsc_images(images):
            col, cr, cb, cg = self._do_single_alsc(image, do_alsc_colour)
            list_col.append(col)
            list_cr.append(cr)
            list_cb.append(cb)
            list_cg.append(cg)
            count += 1

        # Convert to numpy array for data manipulation
        list_col = np.array(list_col)
        list_cr = np.array(list_cr)
        list_cb = np.array(list_cb)
        list_cg = np.array(list_cg)

        cal_cr_list = []
        cal_cb_list = []

        # Note: Calculation of average corners and center of the shading tables
        # has been removed (which ctt had, as it was unused)

        # Average all values for luminance shading and return one table for all temperatures
        lum_lut = list(np.round(np.mean(list_cg, axis=0), 3))

        if not do_alsc_colour:
            return None, None, lum_lut, count

        for ct in sorted(set(list_col)):
            # Average tables for the same colour temperature
            indices = np.where(list_col == ct)
            ct = int(ct)
            t_r = np.round(np.mean(list_cr[indices], axis=0), 3)
            t_b = np.round(np.mean(list_cb[indices], axis=0), 3)

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

        return cal_cr_list, cal_cb_list, lum_lut, count

    # @brief Calculate sigma from two adjacent gain tables
    def _calcSigma(self, g1, g2):
        g1 = np.reshape(g1, self.sector_shape[::-1])
        g2 = np.reshape(g2, self.sector_shape[::-1])

        # Apply gains to gain table
        gg = g1 / g2
        if np.mean(gg) < 1:
            gg = 1 / gg

        # For each internal patch, compute average difference between it and
        # its 4 neighbours, then append to list
        diffs = []
        for i in range(self.sector_shape[1] - 2):
            for j in range(self.sector_shape[0] - 2):
                # Indexing is incremented by 1 since all patches on borders are
                # not counted
                diff = np.abs(gg[i + 1][j + 1] - gg[i][j + 1])
                diff += np.abs(gg[i + 1][j + 1] - gg[i + 2][j + 1])
                diff += np.abs(gg[i + 1][j + 1] - gg[i + 1][j])
                diff += np.abs(gg[i + 1][j + 1] - gg[i + 1][j + 2])
                diffs.append(diff / 4)

        mean_diff = np.mean(diffs)
        return np.round(mean_diff, 5)

    # @brief Obtains sigmas for red and blue, effectively a measure of the
    # 'error'
    def _get_sigma(self, cal_cr_list, cal_cb_list):
        # Provided colour alsc tables were generated for two different colour
        # temperatures sigma is calculated by comparing two calibration temperatures
        # adjacent in colour space

        color_temps = [cal['ct'] for cal in cal_cr_list]

        # Calculate sigmas for each adjacent color_temps and return worst one
        sigma_rs = []
        sigma_bs = []
        for i in range(len(color_temps) - 1):
            sigma_rs.append(self._calcSigma(cal_cr_list[i]['table'], cal_cr_list[i + 1]['table']))
            sigma_bs.append(self._calcSigma(cal_cb_list[i]['table'], cal_cb_list[i + 1]['table']))

        # Return maximum sigmas, not necessarily from the same colour
        # temperature interval
        sigma_r = max(sigma_rs) if sigma_rs else 0.005
        sigma_b = max(sigma_bs) if sigma_bs else 0.005

        return sigma_r, sigma_b

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {
            'omega': 1.3,
            'n_iter': 100,
            'luminance_strength': 0.7
        }

        conf = config[self]
        general_conf = config['general']

        do_alsc_colour = self.do_color.get_value(conf)

        # \todo I have no idea where this input parameter is used
        luminance_strength = self.luminance_strength.get_value(conf)
        if luminance_strength < 0 or luminance_strength > 1:
            luminance_strength = 0.5

        output['luminance_strength'] = luminance_strength

        # \todo Validate images from greyscale camera and force grescale mode
        # \todo Debug functionality

        alsc_out = self._do_all_alsc(images, do_alsc_colour, general_conf)
        # \todo Handle the second green lut
        cal_cr_list, cal_cb_list, luminance_lut, count = alsc_out

        if not do_alsc_colour:
            output['luminance_lut'] = luminance_lut
            output['n_iter'] = 0
            return output

        output['calibrations_Cr'] = cal_cr_list
        output['calibrations_Cb'] = cal_cb_list
        output['luminance_lut'] = luminance_lut

        # The sigmas determine the strength of the adaptive algorithm, that
        # cleans up any lens shading that has slipped through the alsc. These
        # are determined by measuring a 'worst-case' difference between two
        # alsc tables that are adjacent in colour space. If, however, only one
        # colour temperature has been provided, then this difference can not be
        # computed as only one table is available.
        # To determine the sigmas you would have to estimate the error of an
        # alsc table with only the image it was taken on as a check. To avoid
        # circularity, dfault exaggerated sigmas are used, which can result in
        # too much alsc and is therefore not advised.
        # In general, just take another alsc picture at another colour
        # temperature!

        if count == 1:
            output['sigma'] = 0.005
            output['sigma_Cb'] = 0.005
            utils.eprint('Warning: Only one alsc calibration found; standard sigmas used for adaptive algorithm.')
            return output

        # Obtain worst-case scenario residual sigmas
        sigma_r, sigma_b = self._get_sigma(cal_cr_list, cal_cb_list)
        output['sigma'] = np.round(sigma_r, 5)
        output['sigma_Cb'] = np.round(sigma_b, 5)

        return output
