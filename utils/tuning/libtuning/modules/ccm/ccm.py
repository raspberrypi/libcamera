# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas on Board
#
# Base Ccm tuning module

from ..module import Module

from libtuning.ctt_ccm import ccm
import logging

logger = logging.getLogger(__name__)


class CCM(Module):
    type = 'ccm'
    hr_name = 'CCM (Base)'
    out_name = 'GenericCCM'

    def __init__(self, debug: list):
        super().__init__()

        self.debug = debug

    def do_calibration(self, images):
        logger.info('Starting CCM calibration')

        imgs = [img for img in images if img.macbeth is not None]

        # todo: Take LSC calibration results into account.
        cal_cr_list = None
        cal_cb_list = None

        try:
            ccms = ccm(imgs, cal_cr_list, cal_cb_list)
        except ArithmeticError:
            logger.error('CCM calibration failed')
            return None

        return ccms
