import cv2
import numpy as np
import urllib
from block_detection import find_blocks
from Block import Block



class Camera:

    def __init__(self):
        self.url = 'http://192.168.0.20/image/jpeg.cgi'

    def preprocess(self, img):
        # Crop to table
        img = img[100:440, 30:635]

        # Hide robot
        cv2.rectangle(img, (220, 210), (380, 340), 0, -1)

        return img

    def get_blocks(self, show=False):
        img = self.get_raw_image()
        img = self.preprocess(img)
        corners = find_blocks(img)

        if show:
            cv2.imshow('Camera', img), cv2.waitKey(0)

        blocks = []
        for i in range(0, len(corners)):
            blocks.append(Block(corners))

        return blocks

    def get_raw_image(self):
        stream = urllib.urlopen(self.url)
        bytes = ''
        bytes += stream.read(64500)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')

        if a != -1 and b != -1:
            jpg = bytes[a:b+2]
            return cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.CV_LOAD_IMAGE_COLOR)
