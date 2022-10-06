# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# macbeth.py - Locate and extract Macbeth charts from images
# (Copied from: ctt_macbeth_locator.py)

# \todo Add debugging

import cv2
import os
from pathlib import Path
import numpy as np

from libtuning.image import Image


# Reshape image to fixed width without distorting returns image and scale
# factor
def reshape(img, width):
    factor = width / img.shape[0]
    return cv2.resize(img, None, fx=factor, fy=factor), factor


# Correlation function to quantify match
def correlate(im1, im2):
    f1 = im1.flatten()
    f2 = im2.flatten()
    cor = np.corrcoef(f1, f2)
    return cor[0][1]


# @brief Compute coordinates of macbeth chart vertices and square centres
# @return (max_cor, best_map_col_norm, fit_coords, success)
#
# Also returns an error/success message for debugging purposes. Additionally,
# it scores the match with a confidence value.
#
#    Brief explanation of the macbeth chart locating algorithm:
#    - Find rectangles within image
#    - Take rectangles within percentage offset of median perimeter. The
#        assumption is that these will be the macbeth squares
#    - For each potential square, find the 24 possible macbeth centre locations
#        that would produce a square in that location
#    - Find clusters of potential macbeth chart centres to find the potential
#        macbeth centres with the most votes, i.e. the most likely ones
#    - For each potential macbeth centre, use the centres of the squares that
#        voted for it to find macbeth chart corners
#    - For each set of corners, transform the possible match into normalised
#        space and correlate with a reference chart to evaluate the match
#    - Select the highest correlation as the macbeth chart match, returning the
#        correlation as the confidence score
#
# \todo Clean this up
def get_macbeth_chart(img, ref_data):
    ref, ref_w, ref_h, ref_corns = ref_data

    # The code will raise and catch a MacbethError in case of a problem, trying
    # to give some likely reasons why the problem occured, hence the try/except
    try:
        # Obtain image, convert to grayscale and normalise
        src = img
        src, factor = reshape(src, 200)
        original = src.copy()
        a = 125 / np.average(src)
        src_norm = cv2.convertScaleAbs(src, alpha=a, beta=0)

        # This code checks if there are seperate colour channels. In the past the
        # macbeth locator ran on jpgs and this makes it robust to different
        # filetypes. Note that running it on a jpg has 4x the pixels of the
        # average bayer channel so coordinates must be doubled.

        # This is best done in img_load.py in the get_patches method. The
        # coordinates and image width, height must be divided by two if the
        # macbeth locator has been run on a demosaicked image.
        if len(src_norm.shape) == 3:
            src_bw = cv2.cvtColor(src_norm, cv2.COLOR_BGR2GRAY)
        else:
            src_bw = src_norm
        original_bw = src_bw.copy()

        # Obtain image edges
        sigma = 2
        src_bw = cv2.GaussianBlur(src_bw, (0, 0), sigma)
        t1, t2 = 50, 100
        edges = cv2.Canny(src_bw, t1, t2)

        # Dilate edges to prevent self-intersections in contours
        k_size = 2
        kernel = np.ones((k_size, k_size))
        its = 1
        edges = cv2.dilate(edges, kernel, iterations=its)

        # Find contours in image
        conts, _ = cv2.findContours(edges, cv2.RETR_TREE,
                                    cv2.CHAIN_APPROX_NONE)
        if len(conts) == 0:
            raise MacbethError(
                '\nWARNING: No macbeth chart found!'
                '\nNo contours found in image\n'
                'Possible problems:\n'
                '- Macbeth chart is too dark or bright\n'
                '- Macbeth chart is occluded\n'
            )

        # Find quadrilateral contours
        epsilon = 0.07
        conts_per = []
        for i in range(len(conts)):
            per = cv2.arcLength(conts[i], True)
            poly = cv2.approxPolyDP(conts[i], epsilon * per, True)
            if len(poly) == 4 and cv2.isContourConvex(poly):
                conts_per.append((poly, per))

        if len(conts_per) == 0:
            raise MacbethError(
                '\nWARNING: No macbeth chart found!'
                '\nNo quadrilateral contours found'
                '\nPossible problems:\n'
                '- Macbeth chart is too dark or bright\n'
                '- Macbeth chart is occluded\n'
                '- Macbeth chart is out of camera plane\n'
            )

        # Sort contours by perimeter and get perimeters within percent of median
        conts_per = sorted(conts_per, key=lambda x: x[1])
        med_per = conts_per[int(len(conts_per) / 2)][1]
        side = med_per / 4
        perc = 0.1
        med_low, med_high = med_per * (1 - perc), med_per * (1 + perc)
        squares = []
        for i in conts_per:
            if med_low <= i[1] and med_high >= i[1]:
                squares.append(i[0])

        # Obtain coordinates of nomralised macbeth and squares
        square_verts, mac_norm = get_square_verts(0.06)
        # For each square guess, find 24 possible macbeth chart centres
        mac_mids = []
        squares_raw = []
        for i in range(len(squares)):
            square = squares[i]
            squares_raw.append(square)

            # Convert quads to rotated rectangles. This is required as the
            # 'squares' are usually quite irregular quadrilaterls, so
            # performing a transform would result in exaggerated warping and
            # inaccurate macbeth chart centre placement
            rect = cv2.minAreaRect(square)
            square = cv2.boxPoints(rect).astype(np.float32)

            # Reorder vertices to prevent 'hourglass shape'
            square = sorted(square, key=lambda x: x[0])
            square_1 = sorted(square[:2], key=lambda x: x[1])
            square_2 = sorted(square[2:], key=lambda x: -x[1])
            square = np.array(np.concatenate((square_1, square_2)), np.float32)
            square = np.reshape(square, (4, 2)).astype(np.float32)
            squares[i] = square

            # Find 24 possible macbeth chart centres by trasnforming normalised
            # macbeth square vertices onto candidate square vertices found in image
            for j in range(len(square_verts)):
                verts = square_verts[j]
                p_mat = cv2.getPerspectiveTransform(verts, square)
                mac_guess = cv2.perspectiveTransform(mac_norm, p_mat)
                mac_guess = np.round(mac_guess).astype(np.int32)

                mac_mid = np.mean(mac_guess, axis=1)
                mac_mids.append([mac_mid, (i, j)])

        if len(mac_mids) == 0:
            raise MacbethError(
                '\nWARNING: No macbeth chart found!'
                '\nNo possible macbeth charts found within image'
                '\nPossible problems:\n'
                '- Part of the macbeth chart is outside the image\n'
                '- Quadrilaterals in image background\n'
            )

        # Reshape data
        for i in range(len(mac_mids)):
            mac_mids[i][0] = mac_mids[i][0][0]

        # Find where midpoints cluster to identify most likely macbeth centres
        clustering = cluster.AgglomerativeClustering(
            n_clusters=None,
            compute_full_tree=True,
            distance_threshold=side * 2
        )
        mac_mids_list = [x[0] for x in mac_mids]

        if len(mac_mids_list) == 1:
            # Special case of only one valid centre found (probably not needed)
            clus_list = []
            clus_list.append([mac_mids, len(mac_mids)])

        else:
            clustering.fit(mac_mids_list)

            # Create list of all clusters
            clus_list = []
            if clustering.n_clusters_ > 1:
                for i in range(clustering.labels_.max() + 1):
                    indices = [j for j, x in enumerate(clustering.labels_) if x == i]
                    clus = []
                    for index in indices:
                        clus.append(mac_mids[index])
                    clus_list.append([clus, len(clus)])
                clus_list.sort(key=lambda x: -x[1])

            elif clustering.n_clusters_ == 1:
                # Special case of only one cluster found
                clus_list.append([mac_mids, len(mac_mids)])
            else:
                raise MacbethError(
                    '\nWARNING: No macebth chart found!'
                    '\nNo clusters found'
                    '\nPossible problems:\n'
                    '- NA\n'
                )

        # Keep only clusters with enough votes
        clus_len_max = clus_list[0][1]
        clus_tol = 0.7
        for i in range(len(clus_list)):
            if clus_list[i][1] < clus_len_max * clus_tol:
                clus_list = clus_list[:i]
                break
            cent = np.mean(clus_list[i][0], axis=0)[0]
            clus_list[i].append(cent)

        # Get centres of each normalised square
        reference = get_square_centres(0.06)

        # For each possible macbeth chart, transform image into
        # normalised space and find correlation with reference
        max_cor = 0
        best_map = None
        best_fit = None
        best_cen_fit = None
        best_ref_mat = None

        for clus in clus_list:
            clus = clus[0]
            sq_cents = []
            ref_cents = []
            i_list = [p[1][0] for p in clus]
            for point in clus:
                i, j = point[1]

                # Remove any square that voted for two different points within
                # the same cluster. This causes the same point in the image to be
                # mapped to two different reference square centres, resulting in
                # a very distorted perspective transform since cv2.findHomography
                # simply minimises error.
                # This phenomenon is not particularly likely to occur due to the
                # enforced distance threshold in the clustering fit but it is
                # best to keep this in just in case.
                if i_list.count(i) == 1:
                    square = squares_raw[i]
                    sq_cent = np.mean(square, axis=0)
                    ref_cent = reference[j]
                    sq_cents.append(sq_cent)
                    ref_cents.append(ref_cent)

                    # At least four squares need to have voted for a centre in
                    # order for a transform to be found
            if len(sq_cents) < 4:
                raise MacbethError(
                    '\nWARNING: No macbeth chart found!'
                    '\nNot enough squares found'
                    '\nPossible problems:\n'
                    '- Macbeth chart is occluded\n'
                    '- Macbeth chart is too dark of bright\n'
                )

            ref_cents = np.array(ref_cents)
            sq_cents = np.array(sq_cents)

            # Find best fit transform from normalised centres to image
            h_mat, mask = cv2.findHomography(ref_cents, sq_cents)
            if 'None' in str(type(h_mat)):
                raise MacbethError(
                    '\nERROR\n'
                )

            # Transform normalised corners and centres into image space
            mac_fit = cv2.perspectiveTransform(mac_norm, h_mat)
            mac_cen_fit = cv2.perspectiveTransform(np.array([reference]), h_mat)

            # Transform located corners into reference space
            ref_mat = cv2.getPerspectiveTransform(
                mac_fit,
                np.array([ref_corns])
            )
            map_to_ref = cv2.warpPerspective(
                original_bw, ref_mat,
                (ref_w, ref_h)
            )

            # Normalise brigthness
            a = 125 / np.average(map_to_ref)
            map_to_ref = cv2.convertScaleAbs(map_to_ref, alpha=a, beta=0)

            # Find correlation with bw reference macbeth
            cor = correlate(map_to_ref, ref)

            # Keep only if best correlation
            if cor > max_cor:
                max_cor = cor
                best_map = map_to_ref
                best_fit = mac_fit
                best_cen_fit = mac_cen_fit
                best_ref_mat = ref_mat

            # Rotate macbeth by pi and recorrelate in case macbeth chart is
            # upside-down
            mac_fit_inv = np.array(
                ([[mac_fit[0][2], mac_fit[0][3],
                  mac_fit[0][0], mac_fit[0][1]]])
            )
            mac_cen_fit_inv = np.flip(mac_cen_fit, axis=1)
            ref_mat = cv2.getPerspectiveTransform(
                mac_fit_inv,
                np.array([ref_corns])
            )
            map_to_ref = cv2.warpPerspective(
                original_bw, ref_mat,
                (ref_w, ref_h)
            )
            a = 125 / np.average(map_to_ref)
            map_to_ref = cv2.convertScaleAbs(map_to_ref, alpha=a, beta=0)
            cor = correlate(map_to_ref, ref)
            if cor > max_cor:
                max_cor = cor
                best_map = map_to_ref
                best_fit = mac_fit_inv
                best_cen_fit = mac_cen_fit_inv
                best_ref_mat = ref_mat

        # Check best match is above threshold
        cor_thresh = 0.6
        if max_cor < cor_thresh:
            raise MacbethError(
                '\nWARNING: Correlation too low'
                '\nPossible problems:\n'
                '- Bad lighting conditions\n'
                '- Macbeth chart is occluded\n'
                '- Background is too noisy\n'
                '- Macbeth chart is out of camera plane\n'
            )

        # Represent coloured macbeth in reference space
        best_map_col = cv2.warpPerspective(
            original, best_ref_mat, (ref_w, ref_h)
        )
        best_map_col = cv2.resize(
            best_map_col, None, fx=4, fy=4
        )
        a = 125 / np.average(best_map_col)
        best_map_col_norm = cv2.convertScaleAbs(
            best_map_col, alpha=a, beta=0
        )

        # Rescale coordinates to original image size
        fit_coords = (best_fit / factor, best_cen_fit / factor)

        return (max_cor, best_map_col_norm, fit_coords, True)

    # Catch macbeth errors and continue with code
    except MacbethError as error:
        eprint(error)
        return (0, None, None, False)


def find_macbeth(img, mac_config):
    small_chart = mac_config['small']
    show = mac_config['show']

    # Catch the warnings
    warnings.simplefilter("ignore")
    warnings.warn("runtime", RuntimeWarning)

    # Reference macbeth chart is created that will be correlated with the
    # located macbeth chart guess to produce a confidence value for the match.
    script_dir = Path(os.path.realpath(os.path.dirname(__file__)))
    macbeth_ref_path = script_dir.joinpath('macbeth_ref.pgm')
    ref = cv2.imread(str(macbeth_ref_path), flags=cv2.IMREAD_GRAYSCALE)
    ref_w = 120
    ref_h = 80
    rc1 = (0, 0)
    rc2 = (0, ref_h)
    rc3 = (ref_w, ref_h)
    rc4 = (ref_w, 0)
    ref_corns = np.array((rc1, rc2, rc3, rc4), np.float32)
    ref_data = (ref, ref_w, ref_h, ref_corns)

    # Locate macbeth chart
    cor, mac, coords, ret = get_macbeth_chart(img, ref_data)

    # Following bits of code try to fix common problems with simple techniques.
    # If now or at any point the best correlation is of above 0.75, then
    # nothing more is tried as this is a high enough confidence to ensure
    # reliable macbeth square centre placement.

    for brightness in [2, 4]:
        if cor >= 0.75:
            break
        img_br = cv2.convertScaleAbs(img, alpha=brightness, beta=0)
        cor_b, mac_b, coords_b, ret_b = get_macbeth_chart(img_br, ref_data)
        if cor_b > cor:
            cor, mac, coords, ret = cor_b, mac_b, coords_b, ret_b

    # In case macbeth chart is too small, take a selection of the image and
    # attempt to locate macbeth chart within that. The scale increment is
    # root 2

    # These variables will be used to transform the found coordinates at
    # smaller scales back into the original. If ii is still -1 after this
    # section that means it was not successful
    ii = -1
    w_best = 0
    h_best = 0
    d_best = 100

    # d_best records the scale of the best match. Macbeth charts are only looked
    # for at one scale increment smaller than the current best match in order to avoid
    # unecessarily searching for macbeth charts at small scales.
    # If a macbeth chart ha already been found then set d_best to 0
    if cor != 0:
        d_best = 0

    for index, pair in enumerate([{'sel': 2 / 3, 'inc': 1 / 6},
                                  {'sel': 1 / 2, 'inc': 1 / 8},
                                  {'sel': 1 / 3, 'inc': 1 / 12},
                                  {'sel': 1 / 4, 'inc': 1 / 16}]):
        if cor >= 0.75:
            break

        # Check if we need to check macbeth charts at even smaller scales. This
        # slows the code down significantly and has therefore been omitted by
        # default, however it is not unusably slow so might be useful if the
        # macbeth chart is too small to be picked up to by the current
        # subselections.  Use this for macbeth charts with side lengths around
        # 1/5 image dimensions (and smaller...?) it is, however, recommended
        # that macbeth charts take up as large as possible a proportion of the
        # image.
        if index >= 2 and (not small_chart or d_best <= index - 1):
            break

        w, h = list(img.shape[:2])
        # Set dimensions of the subselection and the step along each axis
        # between selections
        w_sel = int(w * pair['sel'])
        h_sel = int(h * pair['sel'])
        w_inc = int(w * pair['inc'])
        h_inc = int(h * pair['inc'])

        loop = ((1 - pair['sel']) / pair['inc']) + 1
        # For each subselection, look for a macbeth chart
        for i in range(loop):
            for j in range(loop):
                w_s, h_s = i * w_inc, j * h_inc
                img_sel = img[w_s:w_s + w_sel, h_s:h_s + h_sel]
                cor_ij, mac_ij, coords_ij, ret_ij = get_macbeth_chart(img_sel, ref_data)

                # If the correlation is better than the best then record the
                # scale and current subselection at which macbeth chart was
                # found. Also record the coordinates, macbeth chart and message.
                if cor_ij > cor:
                    cor = cor_ij
                    mac, coords, ret = mac_ij, coords_ij, ret_ij
                    ii, jj = i, j
                    w_best, h_best = w_inc, h_inc
                    d_best = index + 1

    # Transform coordinates from subselection to original image
    if ii != -1:
        for a in range(len(coords)):
            for b in range(len(coords[a][0])):
                coords[a][0][b][1] += ii * w_best
                coords[a][0][b][0] += jj * h_best

    if not ret:
        return None

    coords_fit = coords
    if cor < 0.75:
        eprint(f'Warning: Low confidence {cor:.3f} for macbeth chart in {img.path.name}')

    if show:
        draw_macbeth_results(img, coords_fit)

    return coords_fit


def locate_macbeth(image: Image, config: dict):
    # Find macbeth centres
    av_chan = (np.mean(np.array(image.channels), axis=0) / (2**16))
    av_val = np.mean(av_chan)
    if av_val < image.blacklevel_16 / (2**16) + 1 / 64:
        eprint(f'Image {image.path.name} too dark')
        return None

    macbeth = find_macbeth(av_chan, config['general']['macbeth'])

    if macbeth is None:
        eprint(f'No macbeth chart found in {image.path.name}')
        return None

    mac_cen_coords = macbeth[1]
    if not image.get_patches(mac_cen_coords):
        eprint(f'Macbeth patches have saturated in {image.path.name}')
        return None

    return macbeth
