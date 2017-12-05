#! /usr/bin/env python
from keras.applications import ResNet50
from keras.applications import Xception
from keras.applications import imagenet_utils
from keras.preprocessing import image
import cv2
import numpy as np

class Res50Classifier:
    def __init__(self):
        self.model = ResNet50(weights='imagenet')

    def load_img(self, img):
        inputShape = (224, 224)
        self.img = cv2.resize(img, inputShape).astype(np.float32)
        self.img = image.img_to_array(self.img)
        self.img = np.expand_dims(self.img, axis=0)
        self.img = imagenet_utils.preprocess_input(self.img)

    def predict(self):
        prediction = self.model.predict(self.img)
        p = imagenet_utils.decode_predictions(prediction)
        top = p[0][0]
        label = top[1].encode('utf-8') # convert unicode to string
        prob = top[2]
        #print type(label)
        return (label, prob)

class XceptionClassifier:
    def __init__(self):
        self.model = Xception(weights='imagenet')

    def load_img(self, img):
        inputShape = (299, 299)
        self.img = cv2.resize(img, inputShape).astype(np.float32)
        self.img = image.img_to_array(self.img)
        self.img = np.expand_dims(self.img, axis=0)
        self.img = imagenet_utils.preprocess_input(self.img)

    def predict(self):
        prediction = self.model.predict(self.img)
        p = imagenet_utils.decode_predictions(prediction)
        top = p[0][0]
        label = top[1].encode('utf-8')  # convert unicode to string
        prob = top[2]
        #print type(label)
        return (label, prob)

