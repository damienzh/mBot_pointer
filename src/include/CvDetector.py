#! /usr/bin/env python
import cv2
import numpy as np
from copy import deepcopy

class CvDetector:
    def __init__(self):
        model = '/home/k/catkin_ws/src/mbot_pointer/param/Caffe/MobileNetSSD_deploy.caffemodel'
        prototxt = '/home/k/catkin_ws/src/mbot_pointer/param/Caffe/MobileNetSSD_deploy.prototxt.txt'
        self.net = cv2.dnn.readNetFromCaffe(prototxt, model)


    def detect(self, img):
        blob = cv2.dnn.blobFromImage(cv2.resize(img, (300, 300)), 0.007843, (300, 300), 127.5)
        self.net.setInput(blob)
        detections = self.net.forward()

        self.objects = []

        CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                   "bottle", "bus", "car", "cat", "chair", "cow", "table",
                   "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                   "sofa", "train", "tvmonitor"]
        COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

        image = deepcopy(img)
        (h, w) = image.shape[:2]

        for i in np.arange(0, detections.shape[2]):
            # extract the confidence (i.e., probability) associated with the
            # prediction
            confidence = detections[0, 0, i, 2]

            # filter out weak detections by ensuring the `confidence` is
            # greater than the minimum confidence
            if confidence > 0.5:
                # extract the index of the class label from the `detections`,
                # then compute the (x, y)-coordinates of the bounding box for
                # the object
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                # display the prediction
                label = CLASSES[idx]
                label_info = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                print("[INFO] {}".format(label_info))
                cv2.rectangle(image, (startX, startY), (endX, endY), COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(image, label_info, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)
                centerX = (startX + endX) / 2
                centerY = (startY + endY) / 2
                newItem = Item()
                newItem.label = label
                newItem.score = confidence
                newItem.box = (startX, startY, endX, endY)
                newItem.center = (centerX, centerY)
                newItem.checkInCenter(w)
                if newItem.inCenter:
                    self.objects.append(newItem)

        return image

class Item:
    def __init__(self):
        self.label = ''
        self.score = 0
        self.box = (0, 0, 0, 0)
        self.center = (0, 0)
        self.inCenter = False

    def checkInCenter(self, imgWidth):
        if self.box[0] < imgWidth/2 and self.box[2] > imgWidth/2:
            self.inCenter = True