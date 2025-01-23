# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2024, Ideas On Board
#
# AWB module for tuning rkisp1

from .awb import AWB

class AWBRkISP1(AWB):
    hr_name = 'AWB (RkISP1)'
    out_name = 'Awb'

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def validate_config(self, config: dict) -> bool:
        return True

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        if not 'awb' in config['general']:
            raise ValueError('AWB configuration missing')
        awb_config = config['general']['awb']
        algorithm = awb_config['algorithm']

        output = {'algorithm': algorithm}
        data = self.do_calculation(images)
        if algorithm == 'grey':
            output['colourGains'] = data['colourGains']
        elif algorithm == 'bayes':
            output['AwbMode'] = awb_config['AwbMode']
            output['priors'] = awb_config['priors']
            output.update(data)
        else:
            raise ValueError(f"Unknown AWB algorithm {output['algorithm']}")

        return output
