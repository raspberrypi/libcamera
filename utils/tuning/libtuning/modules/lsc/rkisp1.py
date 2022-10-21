# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# rkisp1.py - LSC module for tuning rkisp1

from .lsc import LSC

import libtuning as lt
import libtuning.utils as utils

from numbers import Number
import numpy as np


class LSCRkISP1(LSC):
    hr_name = 'LSC (RkISP1)'
    out_name = 'LensShadingCorrection'
    # \todo Not sure if this is useful. Probably will remove later.
    compatible = ['rkisp1']

    def __init__(self, *args, **kwargs):
        super().__init__(**kwargs)

    # We don't actually need anything from the config file
    def validate_config(self, config: dict) -> bool:
        return True

    # @return Image color temperature, flattened array of red calibration table
    #         (containing {sector size} elements), flattened array of blue
    #         calibration table, flattened array of (red's) green calibration
    #         table, flattened array of (blue's) green calibration table

    def _do_single_lsc(self, image: lt.Image):
        cgr, gr = self._lsc_single_channel(image.channels[lt.Color.GR], image)
        cgb, gb = self._lsc_single_channel(image.channels[lt.Color.GB], image)

        # \todo Should these ratio against the average of both greens or just
        # each green like we've done here?
        cr, _ = self._lsc_single_channel(image.channels[lt.Color.R], image, gr)
        cb, _ = self._lsc_single_channel(image.channels[lt.Color.B], image, gb)

        return image.color, cr.flatten(), cb.flatten(), cgr.flatten(), cgb.flatten()

    # @return List of dictionaries of color temperature, red table, red's green
    #         table, blue's green table, and blue table

    def _do_all_lsc(self, images: list) -> list:
        output_list = []
        output_map_func = lt.gradient.Linear().map

        # List of colour temperatures
        list_col = []
        # Associated calibration tables
        list_cr = []
        list_cb = []
        list_cgr = []
        list_cgb = []
        for image in self._enumerate_lsc_images(images):
            col, cr, cb, cgr, cgb = self._do_single_lsc(image)
            list_col.append(col)
            list_cr.append(cr)
            list_cb.append(cb)
            list_cgr.append(cgr)
            list_cgb.append(cgb)

        # Convert to numpy array for data manipulation
        list_col = np.array(list_col)
        list_cr = np.array(list_cr)
        list_cb = np.array(list_cb)
        list_cgr = np.array(list_cgr)
        list_cgb = np.array(list_cgb)

        for color_temperature in sorted(set(list_col)):
            # Average tables for the same colour temperature
            indices = np.where(list_col == color_temperature)
            color_temperature = int(color_temperature)

            tables = []
            for lis in [list_cr, list_cgr, list_cgb, list_cb]:
                table = np.mean(lis[indices], axis=0)
                table = output_map_func((1, 3.999), (1024, 4095), table)
                table = np.round(table).astype('int32').tolist()
                tables.append(table)

            entry = {
                'ct': color_temperature,
                'r': tables[0],
                'gr': tables[1],
                'gb': tables[2],
                'b': tables[3],
            }

            output_list.append(entry)

        return output_list

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {}

        # \todo This should actually come from self.sector_{x,y}_gradient
        size_gradient = lt.gradient.Linear(lt.Remainder.Float)
        output['x-size'] = size_gradient.distribute(0.5, 8)
        output['y-size'] = size_gradient.distribute(0.5, 8)

        output['sets'] = self._do_all_lsc(images)

        # \todo Validate images from greyscale camera and force grescale mode
        # \todo Debug functionality

        return output
