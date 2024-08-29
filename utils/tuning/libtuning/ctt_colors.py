# Program to convert from RGB to LAB color space
def RGB_to_LAB(RGB):  # where RGB is a 1x3 array.   e.g RGB = [100, 255, 230]
    num = 0
    XYZ = [0, 0, 0]
    # converted all the three R, G, B to X, Y, Z
    X = RGB[0] * 0.4124 + RGB[1] * 0.3576 + RGB[2] * 0.1805
    Y = RGB[0] * 0.2126 + RGB[1] * 0.7152 + RGB[2] * 0.0722
    Z = RGB[0] * 0.0193 + RGB[1] * 0.1192 + RGB[2] * 0.9505

    XYZ[0] = X / 255 * 100
    XYZ[1] = Y / 255 * 100  # XYZ Must be in range 0 -> 100, so scale down from 255
    XYZ[2] = Z / 255 * 100
    XYZ[0] = XYZ[0] / 95.047  # ref_X =  95.047   Observer= 2°, Illuminant= D65
    XYZ[1] = XYZ[1] / 100.0  # ref_Y = 100.000
    XYZ[2] = XYZ[2] / 108.883  # ref_Z = 108.883
    num = 0
    for value in XYZ:
        if value > 0.008856:
            value = value ** (0.3333333333333333)
        else:
            value = (7.787 * value) + (16 / 116)
        XYZ[num] = value
        num = num + 1

    # L, A, B, values calculated below
    L = (116 * XYZ[1]) - 16
    a = 500 * (XYZ[0] - XYZ[1])
    b = 200 * (XYZ[1] - XYZ[2])

    return [L, a, b]
