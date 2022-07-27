# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
#
# ctt_macbeth_locator.py - camera tuning tool Macbeth chart locator

from ctt_ransac import *
from ctt_tools import *
import warnings

"""
NOTE: some custom functions have been used here to make the code more readable.
These are defined in tools.py if they are needed for reference.
"""


"""
Some inconsistencies between packages cause runtime warnings when running
the clustering algorithm. This catches these warnings so they don't flood the
output to the console
"""
def fxn():
    warnings.warn("runtime", RuntimeWarning)


"""
Define the success message
"""
success_msg = 'Macbeth chart located successfully'

def find_macbeth(Cam, img, mac_config=(0, 0)):
    small_chart, show = mac_config
    print('Locating macbeth chart')
    Cam.log += '\nLocating macbeth chart'
    """
    catch the warnings
    """
    warnings.simplefilter("ignore")
    fxn()

    """
    Reference macbeth chart is created that will be correlated with the located
    macbeth chart guess to produce a confidence value for the match.
    """
    ref = cv2.imread(Cam.path + 'ctt_ref.pgm', flags=cv2.IMREAD_GRAYSCALE)
    ref_w = 120
    ref_h = 80
    rc1 = (0, 0)
    rc2 = (0, ref_h)
    rc3 = (ref_w, ref_h)
    rc4 = (ref_w, 0)
    ref_corns = np.array((rc1, rc2, rc3, rc4), np.float32)
    ref_data = (ref, ref_w, ref_h, ref_corns)

    """
    locate macbeth chart
    """
    cor, mac, coords, msg = get_macbeth_chart(img, ref_data)

    """
    following bits of code tries to fix common problems with simple
    techniques.
    If now or at any point the best correlation is of above 0.75, then
    nothing more is tried as this is a high enough confidence to ensure
    reliable macbeth square centre placement.
    """

    """
    brighten image 2x
    """
    if cor < 0.75:
        a = 2
        img_br = cv2.convertScaleAbs(img, alpha=a, beta=0)
        cor_b, mac_b, coords_b, msg_b = get_macbeth_chart(img_br, ref_data)
        if cor_b > cor:
            cor, mac, coords, msg = cor_b, mac_b, coords_b, msg_b

    """
    brighten image 4x
    """
    if cor < 0.75:
        a = 4
        img_br = cv2.convertScaleAbs(img, alpha=a, beta=0)
        cor_b, mac_b, coords_b, msg_b = get_macbeth_chart(img_br, ref_data)
        if cor_b > cor:
            cor, mac, coords, msg = cor_b, mac_b, coords_b, msg_b

    """
    In case macbeth chart is too small, take a selection of the image and
    attempt to locate macbeth chart within that. The scale increment is
    root 2
    """
    """
    These variables will be used to transform the found coordinates at smaller
    scales back into the original. If ii is still -1 after this section that
    means it was not successful
    """
    ii = -1
    w_best = 0
    h_best = 0
    d_best = 100
    """
    d_best records the scale of the best match. Macbeth charts are only looked
    for at one scale increment smaller than the current best match in order to avoid
    unecessarily searching for macbeth charts at small scales.
    If a macbeth chart ha already been found then set d_best to 0
    """
    if cor != 0:
        d_best = 0

    """
    scale 3/2 (approx root2)
    """
    if cor < 0.75:
        imgs = []
        """
        get size of image
        """
        shape = list(img.shape[:2])
        w, h = shape
        """
        set dimensions of the subselection and the step along each axis between
        selections
        """
        w_sel = int(2*w/3)
        h_sel = int(2*h/3)
        w_inc = int(w/6)
        h_inc = int(h/6)
        """
        for each subselection, look for a macbeth chart
        """
        for i in range(3):
            for j in range(3):
                w_s, h_s = i*w_inc, j*h_inc
                img_sel = img[w_s:w_s+w_sel, h_s:h_s+h_sel]
                cor_ij, mac_ij, coords_ij, msg_ij = get_macbeth_chart(img_sel, ref_data)
                """
                if the correlation is better than the best then record the
                scale and current subselection at which macbeth chart was
                found. Also record the coordinates, macbeth chart and message.
                """
                if cor_ij > cor:
                    cor = cor_ij
                    mac, coords, msg = mac_ij, coords_ij, msg_ij
                    ii, jj = i, j
                    w_best, h_best = w_inc, h_inc
                    d_best = 1

    """
    scale 2
    """
    if cor < 0.75:
        imgs = []
        shape = list(img.shape[:2])
        w, h = shape
        w_sel = int(w/2)
        h_sel = int(h/2)
        w_inc = int(w/8)
        h_inc = int(h/8)
        for i in range(5):
            for j in range(5):
                w_s, h_s = i*w_inc, j*h_inc
                img_sel = img[w_s:w_s+w_sel, h_s:h_s+h_sel]
                cor_ij, mac_ij, coords_ij, msg_ij = get_macbeth_chart(img_sel, ref_data)
                if cor_ij > cor:
                    cor = cor_ij
                    mac, coords, msg = mac_ij, coords_ij, msg_ij
                    ii, jj = i, j
                    w_best, h_best = w_inc, h_inc
                    d_best = 2

    """
    The following code checks for macbeth charts at even smaller scales. This
    slows the code down significantly and has therefore been omitted by default,
    however it is not unusably slow so might be useful if the macbeth chart
    is too small to be picked up to by the current subselections.
    Use this for macbeth charts with side lengths around 1/5 image dimensions
    (and smaller...?) it is, however, recommended that macbeth charts take up as
    large as possible a proportion of the image.
    """

    if small_chart:

        if cor < 0.75 and d_best > 1:
            imgs = []
            shape = list(img.shape[:2])
            w, h = shape
            w_sel = int(w/3)
            h_sel = int(h/3)
            w_inc = int(w/12)
            h_inc = int(h/12)
            for i in range(9):
                for j in range(9):
                    w_s, h_s = i*w_inc, j*h_inc
                    img_sel = img[w_s:w_s+w_sel, h_s:h_s+h_sel]
                    cor_ij, mac_ij, coords_ij, msg_ij = get_macbeth_chart(img_sel, ref_data)
                    if cor_ij > cor:
                        cor = cor_ij
                        mac, coords, msg = mac_ij, coords_ij, msg_ij
                        ii, jj = i, j
                        w_best, h_best = w_inc, h_inc
                        d_best = 3

        if cor < 0.75 and d_best > 2:
            imgs = []
            shape = list(img.shape[:2])
            w, h = shape
            w_sel = int(w/4)
            h_sel = int(h/4)
            w_inc = int(w/16)
            h_inc = int(h/16)
            for i in range(13):
                for j in range(13):
                    w_s, h_s = i*w_inc, j*h_inc
                    img_sel = img[w_s:w_s+w_sel, h_s:h_s+h_sel]
                    cor_ij, mac_ij, coords_ij, msg_ij = get_macbeth_chart(img_sel, ref_data)
                    if cor_ij > cor:
                        cor = cor_ij
                        mac, coords, msg = mac_ij, coords_ij, msg_ij
                        ii, jj = i, j
                        w_best, h_best = w_inc, h_inc

    """
    Transform coordinates from subselection to original image
    """
    if ii != -1:
        for a in range(len(coords)):
            for b in range(len(coords[a][0])):
                coords[a][0][b][1] += ii*w_best
                coords[a][0][b][0] += jj*h_best

    """
    initialise coords_fit variable
    """
    coords_fit = None
    # print('correlation: {}'.format(cor))
    """
    print error or success message
    """
    print(msg)
    Cam.log += '\n' + msg
    if msg == success_msg:
        coords_fit = coords
        Cam.log += '\nMacbeth chart vertices:\n'
        Cam.log += '{}'.format(2*np.round(coords_fit[0][0]), 0)
        """
        if correlation is lower than 0.75 there may be a risk of macbeth chart
        corners not having been located properly. It might be worth running
        with show set to true to check where the macbeth chart centres have
        been located.
        """
        print('Confidence: {:.3f}'.format(cor))
        Cam.log += '\nConfidence: {:.3f}'.format(cor)
        if cor < 0.75:
            print('Caution: Low confidence guess!')
            Cam.log += 'WARNING: Low confidence guess!'
        # cv2.imshow('MacBeth', mac)
        # represent(mac, 'MacBeth chart')

    """
    extract data from coords_fit and plot on original image
    """
    if show and coords_fit is not None:
        copy = img.copy()
        verts = coords_fit[0][0]
        cents = coords_fit[1][0]

        """
        draw circles at vertices of macbeth chart
        """
        for vert in verts:
            p = tuple(np.round(vert).astype(np.int32))
            cv2.circle(copy, p, 10, 1, -1)
        """
        draw circles at centres of squares
        """
        for i in range(len(cents)):
            cent = cents[i]
            p = tuple(np.round(cent).astype(np.int32))
            """
            draw black circle on white square, white circle on black square an
            grey circle everywhere else.
            """
            if i == 3:
                cv2.circle(copy, p, 8, 0, -1)
            elif i == 23:
                cv2.circle(copy, p, 8, 1, -1)
            else:
                cv2.circle(copy, p, 8, 0.5, -1)
        copy, _ = reshape(copy, 400)
        represent(copy)

    return(coords_fit)


def get_macbeth_chart(img, ref_data):
    """
    function returns coordinates of macbeth chart vertices and square centres,
    along with an error/success message for debugging purposes. Additionally,
    it scores the match with a confidence value.

    Brief explanation of the macbeth chart locating algorithm:
    - Find rectangles within image
    - Take rectangles within percentage offset of median perimeter. The
        assumption is that these will be the macbeth squares
    - For each potential square, find the 24 possible macbeth centre locations
        that would produce a square in that location
    - Find clusters of potential macbeth chart centres to find the potential
        macbeth centres with the most votes, i.e. the most likely ones
    - For each potential macbeth centre, use the centres of the squares that
        voted for it to find macbeth chart corners
    - For each set of corners, transform the possible match into normalised
        space and correlate with a reference chart to evaluate the match
    - Select the highest correlation as the macbeth chart match, returning the
        correlation as the confidence score
    """

    """
    get reference macbeth chart data
    """
    (ref, ref_w, ref_h, ref_corns) = ref_data

    """
    the code will raise and catch a MacbethError in case of a problem, trying
    to give some likely reasons why the problem occred, hence the try/except
    """
    try:
        """
        obtain image, convert to grayscale and normalise
        """
        src = img
        src, factor = reshape(src, 200)
        original = src.copy()
        a = 125/np.average(src)
        src_norm = cv2.convertScaleAbs(src, alpha=a, beta=0)
        """
        This code checks if there are seperate colour channels. In the past the
        macbeth locator ran on jpgs and this makes it robust to different
        filetypes. Note that running it on a jpg has 4x the pixels of the
        average bayer channel so coordinates must be doubled.

        This is best done in img_load.py in the get_patches method. The
        coordinates and image width, height must be divided by two if the
        macbeth locator has been run on a demosaicked image.
        """
        if len(src_norm.shape) == 3:
            src_bw = cv2.cvtColor(src_norm, cv2.COLOR_BGR2GRAY)
        else:
            src_bw = src_norm
        original_bw = src_bw.copy()
        """
        obtain image edges
        """
        sigma = 2
        src_bw = cv2.GaussianBlur(src_bw, (0, 0), sigma)
        t1, t2 = 50, 100
        edges = cv2.Canny(src_bw, t1, t2)
        """
        dilate edges to prevent self-intersections in contours
        """
        k_size = 2
        kernel = np.ones((k_size, k_size))
        its = 1
        edges = cv2.dilate(edges, kernel, iterations=its)
        """
        find Contours in image
        """
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
        """
        find quadrilateral contours
        """
        epsilon = 0.07
        conts_per = []
        for i in range(len(conts)):
            per = cv2.arcLength(conts[i], True)
            poly = cv2.approxPolyDP(conts[i], epsilon*per, True)
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

        """
        sort contours by perimeter and get perimeters within percent of median
        """
        conts_per = sorted(conts_per, key=lambda x: x[1])
        med_per = conts_per[int(len(conts_per)/2)][1]
        side = med_per/4
        perc = 0.1
        med_low, med_high = med_per*(1-perc), med_per*(1+perc)
        squares = []
        for i in conts_per:
            if med_low <= i[1] and med_high >= i[1]:
                squares.append(i[0])

        """
        obtain coordinates of nomralised macbeth and squares
        """
        square_verts, mac_norm = get_square_verts(0.06)
        """
        for each square guess, find 24 possible macbeth chart centres
        """
        mac_mids = []
        squares_raw = []
        for i in range(len(squares)):
            square = squares[i]
            squares_raw.append(square)
            """
            convert quads to rotated rectangles. This is required as the
            'squares' are usually quite irregular quadrilaterls, so performing
            a transform would result in exaggerated warping and inaccurate
            macbeth chart centre placement
            """
            rect = cv2.minAreaRect(square)
            square = cv2.boxPoints(rect).astype(np.float32)
            """
            reorder vertices to prevent 'hourglass shape'
            """
            square = sorted(square, key=lambda x: x[0])
            square_1 = sorted(square[:2], key=lambda x: x[1])
            square_2 = sorted(square[2:], key=lambda x: -x[1])
            square = np.array(np.concatenate((square_1, square_2)), np.float32)
            square = np.reshape(square, (4, 2)).astype(np.float32)
            squares[i] = square
            """
            find 24 possible macbeth chart centres by trasnforming normalised
            macbeth square vertices onto candidate square vertices found in image
            """
            for j in range(len(square_verts)):
                verts = square_verts[j]
                p_mat = cv2.getPerspectiveTransform(verts, square)
                mac_guess = cv2.perspectiveTransform(mac_norm, p_mat)
                mac_guess = np.round(mac_guess).astype(np.int32)
                """
                keep only if candidate macbeth is within image border
                (deprecated)
                """
                in_border = True
                # for p in mac_guess[0]:
                #     pptest = cv2.pointPolygonTest(
                #         img_con,
                #         tuple(p),
                #         False
                #     )
                #     if pptest == -1:
                #         in_border = False
                #         break

                if in_border:
                    mac_mid = np.mean(mac_guess,
                                      axis=1)
                    mac_mids.append([mac_mid, (i, j)])

        if len(mac_mids) == 0:
            raise MacbethError(
                '\nWARNING: No macbeth chart found!'
                '\nNo possible macbeth charts found within image'
                '\nPossible problems:\n'
                '- Part of the macbeth chart is outside the image\n'
                '- Quadrilaterals in image background\n'
            )

        """
        reshape data
        """
        for i in range(len(mac_mids)):
            mac_mids[i][0] = mac_mids[i][0][0]

        """
        find where midpoints cluster to identify most likely macbeth centres
        """
        clustering = cluster.AgglomerativeClustering(
            n_clusters=None,
            compute_full_tree=True,
            distance_threshold=side*2
        )
        mac_mids_list = [x[0] for x in mac_mids]

        if len(mac_mids_list) == 1:
            """
            special case of only one valid centre found (probably not needed)
            """
            clus_list = []
            clus_list.append([mac_mids, len(mac_mids)])

        else:
            clustering.fit(mac_mids_list)
            # try:
            #     clustering.fit(mac_mids_list)
            # except RuntimeWarning as error:
            #     return(0, None, None, error)

            """
            create list of all clusters
            """
            clus_list = []
            if clustering.n_clusters_ > 1:
                for i in range(clustering.labels_.max()+1):
                    indices = [j for j, x in enumerate(clustering.labels_) if x == i]
                    clus = []
                    for index in indices:
                        clus.append(mac_mids[index])
                    clus_list.append([clus, len(clus)])
                clus_list.sort(key=lambda x: -x[1])

            elif clustering.n_clusters_ == 1:
                """
                special case of only one cluster found
                """
                # print('only 1 cluster')
                clus_list.append([mac_mids, len(mac_mids)])
            else:
                raise MacbethError(
                    '\nWARNING: No macebth chart found!'
                    '\nNo clusters found'
                    '\nPossible problems:\n'
                    '- NA\n'
                )

        """
        keep only clusters with enough votes
        """
        clus_len_max = clus_list[0][1]
        clus_tol = 0.7
        for i in range(len(clus_list)):
            if clus_list[i][1] < clus_len_max * clus_tol:
                clus_list = clus_list[:i]
                break
            cent = np.mean(clus_list[i][0], axis=0)[0]
            clus_list[i].append(cent)

        """
        represent most popular cluster centroids
        """
        # copy = original_bw.copy()
        # copy = cv2.cvtColor(copy, cv2.COLOR_GRAY2RGB)
        # copy = cv2.resize(copy, None, fx=2, fy=2)
        # for clus in clus_list:
        #     centroid = tuple(2*np.round(clus[2]).astype(np.int32))
        #     cv2.circle(copy, centroid, 7, (255, 0, 0), -1)
        #     cv2.circle(copy, centroid, 2, (0, 0, 255), -1)
        # represent(copy)

        """
        get centres of each normalised square
        """
        reference = get_square_centres(0.06)

        """
        for each possible macbeth chart, transform image into
        normalised space and find correlation with reference
        """
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
                """
                remove any square that voted for two different points within
                the same cluster. This causes the same point in the image to be
                mapped to two different reference square centres, resulting in
                a very distorted perspective transform since cv2.findHomography
                simply minimises error.
                This phenomenon is not particularly likely to occur due to the
                enforced distance threshold in the clustering fit but it is
                best to keep this in just in case.
                """
                if i_list.count(i) == 1:
                    square = squares_raw[i]
                    sq_cent = np.mean(square, axis=0)
                    ref_cent = reference[j]
                    sq_cents.append(sq_cent)
                    ref_cents.append(ref_cent)

                    """
                    At least four squares need to have voted for a centre in
                    order for a transform to be found
                    """
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
            """
            find best fit transform from normalised centres to image
            """
            h_mat, mask = cv2.findHomography(ref_cents, sq_cents)
            if 'None' in str(type(h_mat)):
                raise MacbethError(
                    '\nERROR\n'
                )

            """
            transform normalised corners and centres into image space
            """
            mac_fit = cv2.perspectiveTransform(mac_norm, h_mat)
            mac_cen_fit = cv2.perspectiveTransform(np.array([reference]), h_mat)
            """
            transform located corners into reference space
            """
            ref_mat = cv2.getPerspectiveTransform(
                mac_fit,
                np.array([ref_corns])
            )
            map_to_ref = cv2.warpPerspective(
                original_bw, ref_mat,
                (ref_w, ref_h)
            )
            """
            normalise brigthness
            """
            a = 125/np.average(map_to_ref)
            map_to_ref = cv2.convertScaleAbs(map_to_ref, alpha=a, beta=0)
            """
            find correlation with bw reference macbeth
            """
            cor = correlate(map_to_ref, ref)
            """
            keep only if best correlation
            """
            if cor > max_cor:
                max_cor = cor
                best_map = map_to_ref
                best_fit = mac_fit
                best_cen_fit = mac_cen_fit
                best_ref_mat = ref_mat

            """
            rotate macbeth by pi and recorrelate in case macbeth chart is
            upside-down
            """
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
            a = 125/np.average(map_to_ref)
            map_to_ref = cv2.convertScaleAbs(map_to_ref, alpha=a, beta=0)
            cor = correlate(map_to_ref, ref)
            if cor > max_cor:
                max_cor = cor
                best_map = map_to_ref
                best_fit = mac_fit_inv
                best_cen_fit = mac_cen_fit_inv
                best_ref_mat = ref_mat

        """
        Check best match is above threshold
        """
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
            """
            Following code is mostly representation for debugging purposes
            """

        """
        draw macbeth corners and centres on image
        """
        copy = original.copy()
        copy = cv2.resize(original, None, fx=2, fy=2)
        # print('correlation = {}'.format(round(max_cor, 2)))
        for point in best_fit[0]:
            point = np.array(point, np.float32)
            point = tuple(2*np.round(point).astype(np.int32))
            cv2.circle(copy, point, 4, (255, 0, 0), -1)
        for point in best_cen_fit[0]:
            point = np.array(point, np.float32)
            point = tuple(2*np.round(point).astype(np.int32))
            cv2.circle(copy, point, 4, (0, 0, 255), -1)
            copy = copy.copy()
            cv2.circle(copy, point, 4, (0, 0, 255), -1)

        """
        represent coloured macbeth in reference space
        """
        best_map_col = cv2.warpPerspective(
            original, best_ref_mat, (ref_w, ref_h)
        )
        best_map_col = cv2.resize(
            best_map_col, None, fx=4, fy=4
        )
        a = 125/np.average(best_map_col)
        best_map_col_norm = cv2.convertScaleAbs(
            best_map_col, alpha=a, beta=0
        )
        # cv2.imshow('Macbeth', best_map_col)
        # represent(copy)

        """
        rescale coordinates to original image size
        """
        fit_coords = (best_fit/factor, best_cen_fit/factor)

        return(max_cor, best_map_col_norm, fit_coords, success_msg)

        """
    catch macbeth errors and continue with code
    """
    except MacbethError as error:
        return(0, None, None, error)
