# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019-2020, Raspberry Pi Ltd
#
# ctt_image_load.py - camera tuning tool image loading

from ctt_tools import *
from ctt_macbeth_locator import *
import json
import pyexiv2 as pyexif
import rawpy as raw


"""
Image class load image from raw data and extracts metadata.

Once image is extracted from data, it finds 24 16x16 patches for each
channel, centred at the macbeth chart squares
"""
class Image:
    def __init__(self, buf):
        self.buf = buf
        self.patches = None
        self.saturated = False

    '''
    obtain metadata from buffer
    '''
    def get_meta(self):
        self.ver = ba_to_b(self.buf[4:5])
        self.w = ba_to_b(self.buf[0xd0:0xd2])
        self.h = ba_to_b(self.buf[0xd2:0xd4])
        self.pad = ba_to_b(self.buf[0xd4:0xd6])
        self.fmt = self.buf[0xf5]
        self.sigbits = 2*self.fmt + 4
        self.pattern = self.buf[0xf4]
        self.exposure = ba_to_b(self.buf[0x90:0x94])
        self.againQ8 = ba_to_b(self.buf[0x94:0x96])
        self.againQ8_norm = self.againQ8/256
        camName = self.buf[0x10:0x10+128]
        camName_end = camName.find(0x00)
        self.camName = self.buf[0x10:0x10+128][:camName_end].decode()

        """
        Channel order depending on bayer pattern
        """
        bayer_case = {
            0: (0, 1, 2, 3),   # red
            1: (2, 0, 3, 1),   # green next to red
            2: (3, 2, 1, 0),   # green next to blue
            3: (1, 0, 3, 2),   # blue
            128: (0, 1, 2, 3)  # arbitrary order for greyscale casw
        }
        self.order = bayer_case[self.pattern]

        '''
        manual blacklevel - not robust
        '''
        if 'ov5647' in self.camName:
            self.blacklevel = 16
        else:
            self.blacklevel = 64
        self.blacklevel_16 = self.blacklevel << (6)
        return 1

    '''
    print metadata for debug
    '''
    def print_meta(self):
        print('\nData:')
        print('      ver = {}'.format(self.ver))
        print('      w = {}'.format(self.w))
        print('      h = {}'.format(self.h))
        print('      pad = {}'.format(self.pad))
        print('      fmt = {}'.format(self.fmt))
        print('      sigbits = {}'.format(self.sigbits))
        print('      pattern = {}'.format(self.pattern))
        print('      exposure = {}'.format(self.exposure))
        print('      againQ8 = {}'.format(self.againQ8))
        print('      againQ8_norm = {}'.format(self.againQ8_norm))
        print('      camName = {}'.format(self.camName))
        print('      blacklevel = {}'.format(self.blacklevel))
        print('      blacklevel_16 = {}'.format(self.blacklevel_16))

        return 1

    """
    get image from raw scanline data
    """
    def get_image(self, raw):
        self.dptr = []
        """
        check if data is 10 or 12 bits
        """
        if self.sigbits == 10:
            """
            calc length of scanline
            """
            lin_len = ((((((self.w+self.pad+3)>>2)) * 5)+31)>>5) * 32
            """
            stack scan lines into matrix
            """
            raw = np.array(raw).reshape(-1, lin_len).astype(np.int64)[:self.h, ...]
            """
            separate 5 bits in each package, stopping when w is satisfied
            """
            ba0 = raw[..., 0:5*((self.w+3)>>2):5]
            ba1 = raw[..., 1:5*((self.w+3)>>2):5]
            ba2 = raw[..., 2:5*((self.w+3)>>2):5]
            ba3 = raw[..., 3:5*((self.w+3)>>2):5]
            ba4 = raw[..., 4:5*((self.w+3)>>2):5]
            """
            assemble 10 bit numbers
            """
            ch0 = np.left_shift((np.left_shift(ba0, 2) + (ba4 % 4)), 6)
            ch1 = np.left_shift((np.left_shift(ba1, 2) + (np.right_shift(ba4, 2) % 4)), 6)
            ch2 = np.left_shift((np.left_shift(ba2, 2) + (np.right_shift(ba4, 4) % 4)), 6)
            ch3 = np.left_shift((np.left_shift(ba3, 2) + (np.right_shift(ba4, 6) % 4)), 6)
            """
            interleave bits
            """
            mat = np.empty((self.h, self.w), dtype=ch0.dtype)

            mat[..., 0::4] = ch0
            mat[..., 1::4] = ch1
            mat[..., 2::4] = ch2
            mat[..., 3::4] = ch3

            """
            There is som eleaking memory somewhere in the code. This code here
            seemed to make things good enough that the code would run for
            reasonable numbers of images, however this is techincally just a
            workaround. (sorry)
            """
            ba0, ba1, ba2, ba3, ba4 = None, None, None, None, None
            del ba0, ba1, ba2, ba3, ba4
            ch0, ch1, ch2, ch3 = None, None, None, None
            del ch0, ch1, ch2, ch3

            """
        same as before but 12 bit case
        """
        elif self.sigbits == 12:
            lin_len = ((((((self.w+self.pad+1)>>1)) * 3)+31)>>5) * 32
            raw = np.array(raw).reshape(-1, lin_len).astype(np.int64)[:self.h, ...]
            ba0 = raw[..., 0:3*((self.w+1)>>1):3]
            ba1 = raw[..., 1:3*((self.w+1)>>1):3]
            ba2 = raw[..., 2:3*((self.w+1)>>1):3]
            ch0 = np.left_shift((np.left_shift(ba0, 4) + ba2 % 16), 4)
            ch1 = np.left_shift((np.left_shift(ba1, 4) + (np.right_shift(ba2, 4)) % 16), 4)
            mat = np.empty((self.h, self.w), dtype=ch0.dtype)
            mat[..., 0::2] = ch0
            mat[..., 1::2] = ch1

        else:
            """
            data is neither 10 nor 12 or incorrect data
            """
            print('ERROR: wrong bit format, only 10 or 12 bit supported')
            return 0

        """
        separate bayer channels
        """
        c0 = mat[0::2, 0::2]
        c1 = mat[0::2, 1::2]
        c2 = mat[1::2, 0::2]
        c3 = mat[1::2, 1::2]
        self.channels = [c0, c1, c2, c3]
        return 1

    """
    obtain 16x16 patch centred at macbeth square centre for each channel
    """
    def get_patches(self, cen_coords, size=16):
        """
        obtain channel widths and heights
        """
        ch_w, ch_h = self.w, self.h
        cen_coords = list(np.array((cen_coords[0])).astype(np.int32))
        self.cen_coords = cen_coords
        """
        squares are ordered by stacking macbeth chart columns from
        left to right. Some useful patch indices:
            white = 3
            black = 23
            'reds' = 9, 10
            'blues' = 2, 5, 8, 20, 22
            'greens' = 6, 12, 17
            greyscale = 3, 7, 11, 15, 19, 23
        """
        all_patches = []
        for ch in self.channels:
            ch_patches = []
            for cen in cen_coords:
                '''
                macbeth centre is placed at top left of central 2x2 patch
                to account for rounding
                Patch pixels are sorted by pixel brightness so spatial
                information is lost.
                '''
                patch = ch[cen[1]-7:cen[1]+9, cen[0]-7:cen[0]+9].flatten()
                patch.sort()
                if patch[-5] == (2**self.sigbits-1)*2**(16-self.sigbits):
                    self.saturated = True
                ch_patches.append(patch)
                # print('\nNew Patch\n')
            all_patches.append(ch_patches)
            # print('\n\nNew Channel\n\n')
        self.patches = all_patches
        return 1


def brcm_load_image(Cam, im_str):
    """
    Load image where raw data and metadata is in the BRCM format
    """
    try:
        """
        create byte array
        """
        with open(im_str, 'rb') as image:
            f = image.read()
            b = bytearray(f)
        """
        return error if incorrect image address
        """
    except FileNotFoundError:
        print('\nERROR:\nInvalid image address')
        Cam.log += '\nWARNING: Invalid image address'
        return 0

    """
    return error if problem reading file
    """
    if f is None:
        print('\nERROR:\nProblem reading file')
        Cam.log += '\nWARNING: Problem readin file'
        return 0

    # print('\nLooking for EOI and BRCM header')
    """
    find end of image followed by BRCM header by turning
    bytearray into hex string and string matching with regexp
    """
    start = -1
    match = bytearray(b'\xff\xd9@BRCM')
    match_str = binascii.hexlify(match)
    b_str = binascii.hexlify(b)
    """
    note index is divided by two to go from string to hex
    """
    indices = [m.start()//2 for m in re.finditer(match_str, b_str)]
    # print(indices)
    try:
        start = indices[0] + 3
    except IndexError:
        print('\nERROR:\nNo Broadcom header found')
        Cam.log += '\nWARNING: No Broadcom header found!'
        return 0
    """
    extract data after header
    """
    # print('\nExtracting data after header')
    buf = b[start:start+32768]
    Img = Image(buf)
    Img.str = im_str
    # print('Data found successfully')

    """
    obtain metadata
    """
    # print('\nReading metadata')
    Img.get_meta()
    Cam.log += '\nExposure : {} us'.format(Img.exposure)
    Cam.log += '\nNormalised gain : {}'.format(Img.againQ8_norm)
    # print('Metadata read successfully')

    """
    obtain raw image data
    """
    # print('\nObtaining raw image data')
    raw = b[start+32768:]
    Img.get_image(raw)
    """
    delete raw to stop memory errors
    """
    raw = None
    del raw
    # print('Raw image data obtained successfully')

    return Img


def dng_load_image(Cam, im_str):
    try:
        Img = Image(None)

        # RawPy doesn't load all the image tags that we need, so we use py3exiv2
        metadata = pyexif.ImageMetadata(im_str)
        metadata.read()

        Img.ver = 100  # random value
        """
        The DNG and TIFF/EP specifications use different IFDs to store the raw
        image data and the Exif tags. DNG stores them in a SubIFD and in an Exif
        IFD respectively (named "SubImage1" and "Photo" by pyexiv2), while
        TIFF/EP stores them both in IFD0 (name "Image"). Both are used in "DNG"
        files, with libcamera-apps following the DNG recommendation and
        applications based on picamera2 following TIFF/EP.

        This code detects which tags are being used, and therefore extracts the
        correct values.
        """
        try:
            Img.w = metadata['Exif.SubImage1.ImageWidth'].value
            subimage = "SubImage1"
            photo = "Photo"
        except KeyError:
            Img.w = metadata['Exif.Image.ImageWidth'].value
            subimage = "Image"
            photo = "Image"
        Img.pad = 0
        Img.h = metadata[f'Exif.{subimage}.ImageLength'].value
        white = metadata[f'Exif.{subimage}.WhiteLevel'].value
        Img.sigbits = int(white).bit_length()
        Img.fmt = (Img.sigbits - 4) // 2
        Img.exposure = int(metadata[f'Exif.{photo}.ExposureTime'].value * 1000000)
        Img.againQ8 = metadata[f'Exif.{photo}.ISOSpeedRatings'].value * 256 / 100
        Img.againQ8_norm = Img.againQ8 / 256
        Img.camName = metadata['Exif.Image.Model'].value
        Img.blacklevel = int(metadata[f'Exif.{subimage}.BlackLevel'].value[0])
        Img.blacklevel_16 = Img.blacklevel << (16 - Img.sigbits)
        bayer_case = {
            '0 1 1 2': (0, (0, 1, 2, 3)),
            '1 2 0 1': (1, (2, 0, 3, 1)),
            '2 1 1 0': (2, (3, 2, 1, 0)),
            '1 0 2 1': (3, (1, 0, 3, 2))
        }
        cfa_pattern = metadata[f'Exif.{subimage}.CFAPattern'].value
        Img.pattern = bayer_case[cfa_pattern][0]
        Img.order = bayer_case[cfa_pattern][1]

        # Now use RawPy tp get the raw Bayer pixels
        raw_im = raw.imread(im_str)
        raw_data = raw_im.raw_image
        shift = 16 - Img.sigbits
        c0 = np.left_shift(raw_data[0::2, 0::2].astype(np.int64), shift)
        c1 = np.left_shift(raw_data[0::2, 1::2].astype(np.int64), shift)
        c2 = np.left_shift(raw_data[1::2, 0::2].astype(np.int64), shift)
        c3 = np.left_shift(raw_data[1::2, 1::2].astype(np.int64), shift)
        Img.channels = [c0, c1, c2, c3]

    except Exception:
        print("\nERROR: failed to load DNG file", im_str)
        print("Either file does not exist or is incompatible")
        Cam.log += '\nERROR: DNG file does not exist or is incompatible'
        raise

    return Img


'''
load image from file location and perform calibration
check correct filetype

mac boolean is true if image is expected to contain macbeth chart and false
if not (alsc images don't have macbeth charts)
'''
def load_image(Cam, im_str, mac_config=None, show=False, mac=True, show_meta=False):
    """
    check image is correct filetype
    """
    if '.jpg' in im_str or '.jpeg' in im_str or '.brcm' in im_str or '.dng' in im_str:
        if '.dng' in im_str:
            Img = dng_load_image(Cam, im_str)
        else:
            Img = brcm_load_image(Cam, im_str)
        """
        handle errors smoothly if loading image failed
        """
        if Img == 0:
            return 0
        if show_meta:
            Img.print_meta()

        if mac:
            """
            find macbeth centres, discarding images that are too dark or light
            """
            av_chan = (np.mean(np.array(Img.channels), axis=0)/(2**16))
            av_val = np.mean(av_chan)
            # print(av_val)
            if av_val < Img.blacklevel_16/(2**16)+1/64:
                macbeth = None
                print('\nError: Image too dark!')
                Cam.log += '\nWARNING: Image too dark!'
            else:
                macbeth = find_macbeth(Cam, av_chan, mac_config)

            """
            if no macbeth found return error
            """
            if macbeth is None:
                print('\nERROR: No macbeth chart found')
                return 0
            mac_cen_coords = macbeth[1]
            # print('\nMacbeth centres located successfully')

            """
            obtain image patches
            """
            # print('\nObtaining image patches')
            Img.get_patches(mac_cen_coords)
            if Img.saturated:
                print('\nERROR: Macbeth patches have saturated')
                Cam.log += '\nWARNING: Macbeth patches have saturated!'
                return 0

        """
        clear memory
        """
        Img.buf = None
        del Img.buf

        # print('Image patches obtained successfully')

        """
        optional debug
        """
        if show and __name__ == '__main__':
            copy = sum(Img.channels)/2**18
            copy = np.reshape(copy, (Img.h//2, Img.w//2)).astype(np.float64)
            copy, _ = reshape(copy, 800)
            represent(copy)

        return Img

        """
    return error if incorrect filetype
    """
    else:
        # print('\nERROR:\nInvalid file extension')
        return 0


"""
bytearray splice to number little endian
"""
def ba_to_b(b):
    total = 0
    for i in range(len(b)):
        total += 256**i * b[i]
    return total
