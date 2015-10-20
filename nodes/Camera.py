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
        x = 0
        y = 80
        img = img[y:y+205, x:x+605]

        # Hide robot
        cv2.rectangle(img, (0, 210), (605, 345), 0, -1)

        return img

    def get_blocks(self):
        img = self.preprocess(self.get_raw_image())
        all_corners = find_blocks(img)

        blocks = []
        for i in range(0, len(all_corners)):
            block = Block(all_corners[i], img)

            for j in range(0, 4):
                cv2.circle(img, (all_corners[i][j][0], all_corners[i][j][1]), 3, (0, 0, 0), -1)

            blocks.append(block)

        cv2.imshow('Camera', img), cv2.waitKey(0)

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
