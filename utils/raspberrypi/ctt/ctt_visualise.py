"""
Some code that will save virtual macbeth charts that show the difference between optimised matrices and non optimised matrices

The function creates an image that is 1550 by 1050 pixels wide, and fills it with patches which are 200x200 pixels in size
Each patch contains the ideal color, the color from the original matrix, and the color from the final matrix
_________________
|               |
|  Ideal Color  |
|_______________|
|  Old  |  new  |
| Color | Color |
|_______|_______|

Nice way of showing how the optimisation helps change the colors and the color matricies
"""
import numpy as np
from PIL import Image


def visualise_macbeth_chart(macbeth_rgb, original_rgb, new_rgb, output_filename):
    image = np.zeros((1050, 1550, 3), dtype=np.uint8)
    colorindex = -1
    for y in range(6):
        for x in range(4):  # Creates 6 x 4 grid of macbeth chart
            colorindex += 1
            xlocation = 50 + 250 * x  # Means there is 50px of black gap between each square, more like the real macbeth chart.
            ylocation = 50 + 250 * y
            for g in range(200):
                for i in range(100):
                    image[xlocation + i, ylocation + g] = macbeth_rgb[colorindex]
            xlocation = 150 + 250 * x
            ylocation = 50 + 250 * y
            for i in range(100):
                for g in range(100):
                    image[xlocation + i, ylocation + g] = original_rgb[colorindex]  # Smaller squares below to compare the old colors with the new ones
            xlocation = 150 + 250 * x
            ylocation = 150 + 250 * y
            for i in range(100):
                for g in range(100):
                    image[xlocation + i, ylocation + g] = new_rgb[colorindex]

    img = Image.fromarray(image, 'RGB')
    img.save(str(output_filename) + 'Generated Macbeth Chart.png')
