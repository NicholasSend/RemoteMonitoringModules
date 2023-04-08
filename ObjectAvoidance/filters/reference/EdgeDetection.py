"""
Goal: Write a function that finds edge intensity and orientation in an image.

Author: Nicholas Sendyk
"""
import math

import cv2
import numpy as np


def edge_detection(img, sigma=1):
    """
    Detects edges in images

    :param img:
    :param sigma:
    :return:
    """
    # Calculate the size of the Gaussian filter based on sigma
    hsize = 2 * int(np.ceil(3 * sigma)) + 1

    # Create the Gaussian filter
    kernel = cv2.getGaussianKernel(hsize, sigma)
    img = cv2.filter2D(src=img, ddepth=-1, kernel=kernel)

    # Calculate gradient in x and y direction
    horizontal_filter = np.array([[1, 0, -1], [2, 0, -2], [1, 0, -1]])
    vertical_filter = np.array([[1, 2, 1], [0, 0, 0], [-1, -2, -1]])

    # Apply Sobel and store results in imgx and imgy
    imgx = cv2.filter2D(img, cv2.CV_64F, horizontal_filter)
    imgy = cv2.filter2D(img, cv2.CV_64F, vertical_filter)

    # Initialize gradient magnitude/orientation matrices
    g_magnitude = np.zeros(imgx.shape)
    g_orientation = np.zeros(imgy.shape)
    orientation_img = np.zeros(imgx.shape)

    # Calculate the Gradient Orientation and Gradient Magnitude of the image
    rows, cols = imgx.shape
    for x in range(rows):
        for y in range(cols):
            g_x = imgx[x][y]
            g_y = imgy[x][y]
            magnitude = float((g_x ** 2 + g_y ** 2) ** (1 / 2))
            angle = (round(np.rad2deg(math.atan(float(g_x) / (g_y + 0.00001))) / 45) % 4) * 45

            # Apply Simple Thresholding
            if magnitude > 35:
                g_magnitude[x][y] = magnitude
                g_orientation[x][y] = angle
            orientation_img[x][y] = ((angle / 45) + 1) * (255 / 5)

    # Apply Non-Maximum Suppression to the Gradient Magnitude
    non_max_sup = g_magnitude.copy()
    for x in range(rows):
        for y in range(cols):
            if g_magnitude[x][y] != 0:
                angle = g_orientation[x][y]
                magnitude = g_magnitude[x][y]

                match angle:
                    case 0:
                        if magnitude <= g_magnitude[x - 1][y] or magnitude <= g_magnitude[x + 1][y]:
                            non_max_sup[x][y] = 0
                    case 45:
                        if magnitude <= g_magnitude[x - 1][y - 1] or magnitude <= g_magnitude[x + 1][y + 1]:
                            non_max_sup[x][y] = 0
                    case 90:
                        if magnitude <= g_magnitude[x][y - 1] or magnitude <= g_magnitude[x][y + 1]:
                            non_max_sup[x][y] = 0
                    case 135:
                        if magnitude <= g_magnitude[x - 1][y + 1] or magnitude <= g_magnitude[x + 1][y - 1]:
                            non_max_sup[x][y] = 0
                    case _:
                        print(f"Unexpected input: [{x},{y}] {angle}")

    # Output Resulting Images
    cv2.imwrite(f'./gradient_magnitude.jpg', g_magnitude)
    cv2.imwrite(f'./edge_detection.jpg', non_max_sup)
    cv2.imwrite(f'./gradient_orientation.jpg', orientation_img)

    return non_max_sup


if __name__ == '__main__':
    # Load image and convert to grayscale
    img0 = cv2.imread("img0.jpg", cv2.IMREAD_GRAYSCALE)

    # Call edge_detection function
    edges = edge_detection(img0, sigma=2)
