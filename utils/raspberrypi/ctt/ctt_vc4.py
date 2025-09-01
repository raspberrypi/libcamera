#!/usr/bin/env python3
#
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_vc4.py - camera tuning tool data for VC4 platforms


json_template = {
    "rpi.black_level": {
        "black_level": 4096
    },
    "rpi.dpc": {
    },
    "rpi.lux": {
        "reference_shutter_speed": 10000,
        "reference_gain": 1,
        "reference_aperture": 1.0
    },
    "rpi.noise": {
    },
    "rpi.geq": {
    },
    "rpi.sdn": {
    },
    "rpi.awb": {
        "priors": [
            {"lux": 0, "prior": [2000, 1.0, 3000, 0.0, 13000, 0.0]},
            {"lux": 800, "prior": [2000, 0.0, 6000, 2.0, 13000, 2.0]},
            {"lux": 1500, "prior": [2000, 0.0, 4000, 1.0, 6000, 6.0, 6500, 7.0, 7000, 1.0, 13000, 1.0]}
        ],
        "modes": {
            "auto": {"lo": 2500, "hi": 8000},
            "incandescent": {"lo": 2500, "hi": 3000},
            "tungsten": {"lo": 3000, "hi": 3500},
            "fluorescent": {"lo": 4000, "hi": 4700},
            "indoor": {"lo": 3000, "hi": 5000},
            "daylight": {"lo": 5500, "hi": 6500},
            "cloudy": {"lo": 7000, "hi": 8600}
        },
        "bayes": 1
    },
    "rpi.agc": {
        "metering_modes": {
            "centre-weighted": {
                "weights": [3, 3, 3, 2, 2, 2, 2, 1, 1, 1, 1, 0, 0, 0, 0]
            },
            "spot": {
                "weights": [2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
            },
            "matrix": {
                "weights": [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
            }
        },
        "exposure_modes": {
            "normal": {
                "shutter": [100, 10000, 30000, 60000, 66666],
                "gain": [1.0, 2.0, 4.0, 6.0, 8.0]
            },
            "short": {
                "shutter": [100, 5000, 10000, 20000, 66666],
                "gain": [1.0, 2.0, 4.0, 6.0, 8.0]
            },
            "long": {
                "shutter": [ 100, 10000, 30000, 60000, 120000 ],
                "gain": [ 1.0, 2.0, 4.0, 6.0, 8.0 ]
            }
        },
        "constraint_modes": {
            "normal": [
                {"bound": "LOWER", "q_lo": 0.98, "q_hi": 1.0, "y_target": [0, 0.5, 1000, 0.5]}
            ],
            "highlight": [
                {"bound": "LOWER", "q_lo": 0.98, "q_hi": 1.0, "y_target": [0, 0.5, 1000, 0.5]},
                {"bound": "UPPER", "q_lo": 0.98, "q_hi": 1.0, "y_target": [0, 0.8, 1000, 0.8]}
            ]
        },
        "y_target": [0, 0.16, 1000, 0.165, 10000, 0.17]
    },
    "rpi.alsc": {
        'omega': 1.3,
        'n_iter': 100,
        'luminance_strength': 0.7,
    },
    "rpi.contrast": {
        "ce_enable": 1,
        "gamma_curve": [
            0,     0,
            1024,  5040,
            2048,  9338,
            3072,  12356,
            4096,  15312,
            5120,  18051,
            6144,  20790,
            7168,  23193,
            8192,  25744,
            9216,  27942,
            10240, 30035,
            11264, 32005,
            12288, 33975,
            13312, 35815,
            14336, 37600,
            15360, 39168,
            16384, 40642,
            18432, 43379,
            20480, 45749,
            22528, 47753,
            24576, 49621,
            26624, 51253,
            28672, 52698,
            30720, 53796,
            32768, 54876,
            36864, 57012,
            40960, 58656,
            45056, 59954,
            49152, 61183,
            53248, 62355,
            57344, 63419,
            61440, 64476,
            65535, 65535
        ]
    },
    "rpi.ccm": {
    },
    "rpi.sharpen": {
        "threshold": 0.75,
        "limit": 0.5,
        "strength": 1.0
    }
}

grid_size = (16, 12)
