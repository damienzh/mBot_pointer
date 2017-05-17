import cv2
import numpy as np
import matplotlib.pyplot as plt
import os
from config import *

# X_RANGE = slice()
ROW_RANGE = slice(480/2-60, 480/2+60)


def depth_process(img):

    img[img > R200_MAX_DISTANCE] = 0
    img[img < R200_MIN_DISTANCE] = 0
    # img = img[ROW_RANGE, :]
    img = depth_filter(img, 0)
    im_d = depth_filter(img, 2)
    img_c = depth_filter(img, 5)
    bins = np.linspace(R200_MIN_DISTANCE, R200_MAX_DISTANCE, 100)
    hist = cv2.calcHist([img_c], [0], None, [100], [R200_MIN_DISTANCE, R200_MAX_DISTANCE])
    segs = histo_segment(hist, 100)
    hist_g = np.gradient(hist, axis=0)
    print(np.median(hist))
    # col_median = np.median(im, axis=0)
    # row_median = np.median(im, axis=1)

    im_gx = cv2.Sobel(img_c, cv2.CV_16U, dx=1, dy=0, ksize=-1)
    im_gy = cv2.Sobel(img_c, cv2.CV_16U, dx=0, dy=1, ksize=-1)

    img_g = cv2.subtract(im_gx, im_gy)
    plt.figure(1)
    plt.plot(hist)
    plt.figure(2)
    plt.imshow(img_g)
    plt.show()


def depth_thresh(img, low_thresh, high_thresh, convert=False):
    img[img < low_thresh] = 0
    img[img > high_thresh] = 0
    if convert:
        img = np.uint8(img)

    return img


def histo_segment(hist, thresh_low):

    hist = np.reshape(hist, -1)
    # find all the indices above low threshold
    inds = np.nonzero(hist > thresh_low)[0]
    segments = np.split(inds, np.where(np.diff(inds) != 1)[0] + 1)

    return segments


def depth_filter(img, option):
    """
    :param img:  uint16 depth image
    :param option:  0: median filter
                    1: Gaussian filter
                    2: dilation
                    3: erosion
                    4: opening
                    5: closing
    :return: 
    """
    if option == 0 or option == 'median':
        img_p = cv2.medianBlur(img, ksize=5)
    elif option == 1 or option == 'gaussian':
        img_p = cv2.GaussianBlur(img, (5, 5), 0)
    elif option == 2 or option == 'dilation':
        img_p = cv2.dilate(img, kernel=np.ones((5, 5), np.uint16), iterations=1)
    elif option == 3 or option == 'erosion':
        img_p = cv2.erode(img, kernel=np.ones((5, 5), np.uint16), iterations=1)
    elif option == 4 or option == 'opening':
        img_p = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel=np.ones((5, 5), np.uint16))
    elif option == 5 or option == 'closing':
        img_p = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel=np.ones((5, 5), np.uint16))

    return img_p


def color_process(img):

    img_h = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # img_l = cv2.cvtColor(img, cv2.COLOR_BGR2LUV)
    # int(img_l.dtype)
    cv2.imshow('color in hsv', img_h)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def scanline(img, subWindow):

    s = np.mean(img[subWindow], axis=0)
    return s


def remove_ground(d_img, height):
    V_FOV = 46  # degree
    H_FOV = 59  # degree
    v_num, h_num = d_img.shape
    v_rad = np.radians(np.linspace(V_FOV/2, -V_FOV/2, v_num))
    h_rad = np.radians(np.linspace(-H_FOV/2, H_FOV/2, h_num))
    b = height / np.tan(v_rad)
    a = b * np.tan(h_rad)
    aa, bb = np.meshgrid(a, b)
    cc = np.ones(aa.shape) * height
    dd = np.sqrt(aa**2 + bb**2 + cc**2)

    return dd


def depth2image(img):
    """convert uint16 depth image to uint8 image"""
    img_8 = np.uint8(np.float32(img) * 255 / np.max(img))
    return img_8


def imread(file, xtion=True):
    if os.path.isfile(file):
        img = cv2.imread(file, -1)
        if xtion:
            img = img[:, 0:631]
    else:
        raise IOError("File does not exist")

    return img


if __name__ == '__main__':
    imd_filename = 'test_data/R200/depth_image_dc4_20170408-170421.png'  # box
    imd_filename = 'test_data/R200/depth_image_dc4_20170408-171930.png'  # doorway
    imd_filename = 'test_data/R200/depth_image_dc4_20170408-171342.png'  # table
    imc_filename = 'test_data/R200/color_image20170408-170543.png'

    xtion_depth = ['xtion_depth_image_1.png', 'xtion_depth_image_2.png', 'xtion_depth_image_3.png',
                   'xtion_depth_image_4.png']
    xtion_color = ['xtion_color_image_1.png', 'xtion_color_image_2.png', 'xtion_color_image_3.png',
                   'xtion_color_image_4.png']

    path = 'test_data/xtion/'
    # print path + xtion_depth[0]
    # img1 = cv2.imread(path+xtion_depth[0], -1)
    img2 = imread(path+xtion_depth[1])
    img2_8 = depth2image(img2)
    edges2 = cv2.Canny(img2_8, 50, 200)
    lines2 = cv2.HoughLinesP(edges2, 1, np.pi / 180, 100, minLineLength=100, maxLineGap=10)
    for line in lines2:
        x1, y1, x2, y2 = line[0]
        cv2.line(img2_8, (x1, y1), (x2, y2), (0, 255, 0), 2)

    cv2.imshow('edges', img2_8)
    cv2.waitKey(0)
    cv2.destroyAllWindows()