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


def process_colorimage(img):

    img_h = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    cv2.imshow('color in hsv', img_h)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    im_filename = 'Images/depth_image_table3.png'
    imc_filename = 'Images/color_image_table3.png'
    im_d = cv2.imread(im_filename, -1)
    im_c = cv2.imread(imc_filename, 3)

    depth_process(im_d)
    #process_colorimage(im_c)