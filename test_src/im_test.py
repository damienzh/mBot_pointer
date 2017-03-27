import cv2
import numpy as np

def depth_process(img):
    pass


if __name__ == '__main__':
    im_filename = 'Images/depth_image1.png'
    im_d = cv2.imread(im_filename, -1)

    depth_process(im_d)