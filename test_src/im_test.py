import cv2
import numpy as np
import matplotlib.pyplot as plt

MAX_DISTANCE = 4000
MIN_DISTANCE = 350
# X_RANGE = slice()
ROW_RANGE = slice(480/2-60, 480/2+60)


def depth_process(img):

    img[img > MAX_DISTANCE] = 0
    img[img < MIN_DISTANCE] = 0
    # img = img[ROW_RANGE, :]
    im = cv2.medianBlur(img, 5)
    im_d = cv2.dilate(im, kernel=np.ones((5, 5), np.uint16), iterations=1)
    col_median = np.median(im, axis=0)
    row_median = np.median(im, axis=1)

    im_gx = cv2.Sobel(img, cv2.CV_16U, dx=1, dy=0, ksize=5)
    im_gy = cv2.Sobel(img, cv2.CV_16U, dx=0, dy=1, ksize=5)

    plt.figure(1)
    plt.subplot(211)
    plt.plot(col_median)
    plt.title('column median')
    plt.subplot(212)
    plt.plot(row_median)
    plt.title('row median')
    plt.figure(2)
    plt.imshow(im_d)
    plt.show()

def show_depth(img, depth_resolution, xy_resolution):

    size = img.shape
    row = size[0]
    col = size[1]
    depth_step = (MAX_DISTANCE - MIN_DISTANCE)/depth_resolution
    x_step = col / xy_resolution
    y_step = row / xy_resolution


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

if __name__ == '__main__':
    imd_filename = 'test_data/depth_image_dc4_20170408-170421.png'
    imc_filename = 'test_data/color_image20170408-170543.png'
    im_d = cv2.imread(imd_filename, -1)
    im_c = cv2.imread(imc_filename, 3)

    # depth_process(im_d)
    color_process(im_c)