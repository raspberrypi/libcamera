# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2023, Raspberry Pi Ltd
#
# generate_filter.py - Resample filter kernel generation
#
# Refer to the PiSP specification, for the register details

import numpy as np
import argparse


# Lanczos (order, length)
# Order is typically 2 or 3
def lanczos(order, N):
    x = np.linspace(-order, +order, N + 6, dtype=np.float64)
    h = np.where((x > -order) & (x < order), np.sinc(x) * np.sinc(x / order), 0)
    return h[3:-3]


# Mitchell - Netravali filters (B, C, length)
# B,C = (1,0) is the cubic B-spline
# B,C = (1/3,1/3) is a good compromise
# B,C = (0, 0.5) for Catmull-Rom
def mitchell(B, C, N):
    x = np.linspace(-2, +2, N, dtype=np.float64)
    h = np.zeros(N)
    for i in range(N):
        ax = abs(x[i])
        if ax < 1:
            h[i] = ((12 - 9 * B - 6 * C) * ax**3 + (-18 + 12 * B + 6 * C) * ax**2 + (6 - 2 * B)) / 6
        elif (ax >= 1) and (ax < 2):
            h[i] = ((-B - 6 * C) * ax**3 + (6 * B + 30 * C) * ax**2 + (-12 * B - 48 * C) * ax + (8 * B + 24 * C)) / 6
    return h


# bicubic_spline(alpha, length);
# alpha is typically set to -0.5 (Hermite spline) or -0.75
def bicubic_spline(a, N):
    x = np.linspace(-2, +2, N, dtype=np.float64)
    h = np.zeros(N)
    for i in range(N):
        ax = abs(x[i])
        if ax <= 1:
            h[i] = (a + 2) * ax**3 - (a + 3) * ax**2 + 1
        elif (ax > 1) and (ax < 2):
            h[i] = (a) * ax**3 - (5 * a) * ax**2 + (8 * a) * ax - (4 * a)
    return h


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--phases', metavar='P', type=int, help='Number of phases.', default=16)
    parser.add_argument('--taps', metavar='T', type=int, help='Number of filter taps per phase.', default=6)
    parser.add_argument('--precision', metavar='PR', type=int, help='Filter precision required.', default=10)
    parser.add_argument('--filter', type=str, metavar='F',
                        help='Filter type and parameters, e.g.: \n'
                        '"Mitchell, b = 0.333, c = 0.333"\n'
                        '"Lanczos, order = 3"\n'
                        '"bicubic_spline, a=-0.5"', required=True)

    args = parser.parse_args()

    phases = args.phases
    taps = args.taps
    precision = args.precision

    # Parse the filter string and pick out the needed parameters.
    filt = args.filter.split(',')
    params = {'a': 0., 'b': 0., 'c': 0., 'order': 0}
    for param in filt[1:]:
        p = param.replace(' ', '').split('=')
        params[p[0]] = type(params[p[0]])(p[1])

    # Generate the filter.
    if (filt[0].lower() == 'mitchell'):
        filter = f'"Michell - Netravali (B = {params["b"]:.3f}, C = {params["c"]:.3f})": [\n'
        h = mitchell(params['b'], params['c'], phases * taps)
    elif (filt[0].lower() == 'lanczos'):
        filter = f'"Lanczos order {params["order"]}": [\n'
        h = lanczos(params['order'], phases * taps)
    elif (filt[0].lower() == 'bicubic_spline'):
        filter = f'"Bicubic-spline (a = {params["a"]:.3f})": [\n'
        h = bicubic_spline(params['a'], phases * taps)
    else:
        print(f'Invalid filter ({filt[0]}) selected!')
        exit()

    # Normalise and convert to fixed-point.
    h = h * phases / np.sum(h)
    h = np.rint(h * (1 << precision)).astype(np.int32)

    ppf = np.zeros((phases, taps), dtype=np.int32)
    for i in range(phases):
        # Pick out phases and flip array.
        ppf[i] = (h[i::phases])[::-1]
        # Make sure there is no DC change by adjusting the largest coefficients.
        max_index = np.nonzero(ppf[i] == ppf[i].max())[0]
        ppf[i, max_index] += (1 << precision) - np.int32(ppf[i].sum() / max_index.size)

    nl = '\n'
    for i in range(phases):
        phase = ', '.join([f'{c:>4}' for c in ppf[i, :]])
        filter += f'    {phase}{nl+"]" if i==phases-1 else ","+nl}'

    print(filter)


if __name__ == '__main__':
    main()
