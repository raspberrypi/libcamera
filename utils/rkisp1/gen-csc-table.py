#!/usr/bin/python3
# SPDX-License-Identifier: GPL-2.0-or-later
# Copyright (C) 2022, Ideas on Board Oy
#
# Generate color space conversion table coefficients with configurable
# fixed-point precision

import argparse
import enum
import sys


encodings = {
    'rec601': [
        [  0.2990,  0.5870,  0.1140 ],
        [ -0.1687, -0.3313,  0.5    ],
        [  0.5,    -0.4187, -0.0813 ]
    ],
    'rec709': [
        [  0.2126,  0.7152,  0.0722 ],
        [ -0.1146, -0.3854,  0.5    ],
        [  0.5,    -0.4542, -0.0458 ]
    ],
    'rec2020': [
        [  0.2627,  0.6780,  0.0593 ],
        [ -0.1396, -0.3604,  0.5    ],
        [  0.5,    -0.4598, -0.0402 ]
    ],
    'smpte240m': [
        [  0.2122,  0.7013,  0.0865 ],
        [ -0.1161, -0.3839,  0.5    ],
        [  0.5,    -0.4451, -0.0549 ]
    ],
}


class Precision(object):
    def __init__(self, precision):
        if precision[0].upper() != 'Q':
            raise RuntimeError(f'Invalid precision `{precision}`')
        prec = precision[1:].split('.')
        if len(prec) != 2:
            raise RuntimeError(f'Invalid precision `{precision}`')

        self.__prec = [int(v) for v in prec]

    @property
    def integer(self):
        return self.__prec[0]

    @property
    def fractional(self):
        return self.__prec[1]

    @property
    def total(self):
        # Add 1 for the sign bit
        return self.__prec[0] + self.__prec[1] + 1


class Quantization(enum.Enum):
    FULL = 0
    LIMITED = 1


def scale_coeff(coeff, quantization, luma, precision):
    """Scale a coefficient to the output range dictated by the quantization and
    the precision.

    Parameters
    ----------
    coeff : float
        The CSC matrix coefficient to scale
    quantization : Quantization
        The quantization, either FULL or LIMITED
    luma : bool
        True if the coefficient corresponds to a luma value, False otherwise
    precision : int
        The desired precision for the scaled coefficient as a number of
        fractional bits
    """

    # Assume the input range is 8 bits. The output range is set by the
    # quantization and differs between luma and chrome components for limited
    # range.
    in_range = 255 - 0
    if quantization == Quantization.FULL:
        out_range = 255 - 0
    elif luma:
        out_range = 235 - 16
    else:
        out_range = 240 - 16

    return coeff * out_range / in_range * (1 << precision)


def round_array(values):
    """Round a list of signed floating point values to the closest integer while
    preserving the (rounded) value of the sum of all elements.
    """

    # Calculate the rounding error as the difference between the rounded sum of
    # values and the sum of rounded values. This is by definition an integer
    # (positive or negative), which indicates how many values will need to be
    # 'flipped' to the opposite rounding.
    rounded_values = [round(value) for value in values]
    sum_values = round(sum(values))
    sum_error = sum_values - sum(rounded_values)

    if sum_error == 0:
        return rounded_values

    # The next step is to distribute the error among the values, in a way that
    # will minimize the relative error introduced in individual values. We
    # extend the values list with the rounded value and original index for each
    # element, and sort by rounding error. Then we modify the elements with the
    # highest or lowest error, depending on whether the sum error is negative
    # or positive.

    values = [[value, round(value), index] for index, value in enumerate(values)]
    values.sort(key=lambda v: v[1] - v[0])

    # It could also be argued that the key for the sort order should not be the
    # absolute rouding error but the relative error, as the impact of identical
    # rounding errors will differ for coefficients with widely different values.
    # This is a topic for further research.
    #
    # values.sort(key=lambda v: (v[1] - v[0]) / abs(v[0]))

    if sum_error > 0:
        for i in range(sum_error):
            values[i][1] += 1
    else:
        for i in range(-sum_error):
            values[len(values) - i - 1][1] -= 1

    # Finally, sort back by index, make sure the total rounding error is now 0,
    # and return the rounded values.
    values.sort(key=lambda v: v[2])
    values = [value[1] for value in values]
    assert(sum(values) == sum_values)

    return values


def main(argv):

    # Parse command line arguments.
    parser = argparse.ArgumentParser(
        description='Generate color space conversion table coefficients with '
        'configurable fixed-point precision.'
    )
    parser.add_argument('--precision', '-p', default='Q1.7',
                        help='The output fixed point precision in Q notation (sign bit excluded)')
    parser.add_argument('--quantization', '-q', choices=['full', 'limited'],
                        default='limited', help='Quantization range')
    parser.add_argument('encoding', choices=encodings.keys(), help='YCbCr encoding')
    args = parser.parse_args(argv[1:])

    try:
        precision = Precision(args.precision)
    except Exception:
        print(f'Invalid precision `{args.precision}`')
        return 1

    encoding = encodings[args.encoding]
    quantization = Quantization[args.quantization.upper()]

    # Scale and round the encoding coefficients based on the precision and
    # quantization range.
    luma = True
    scaled_coeffs = []
    for line in encoding:
        line = [scale_coeff(coeff, quantization, luma, precision.fractional) for coeff in line]
        scaled_coeffs.append(line)
        luma = False

    rounded_coeffs = []
    for line in scaled_coeffs:
        line = round_array(line)

        # Convert coefficients to the number of bits selected by the precision.
        # Negative values will be turned into positive integers using 2's
        # complement.
        line = [coeff & ((1 << precision.total) - 1) for coeff in line]
        rounded_coeffs.append(line)

    # Print the result as C code.
    nbits = 1 << (precision.total - 1).bit_length()
    nbytes = nbits // 4
    print(f'static const u{nbits} rgb2yuv_{args.encoding}_{quantization.name.lower()}_coeffs[] = {{')

    for line in rounded_coeffs:
        line = [f'0x{coeff:0{nbytes}x}' for coeff in line]

        print(f'\t{", ".join(line)},')

    print('};')

    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
