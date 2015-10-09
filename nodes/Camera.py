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
        y = 48
        img = img[y:y+345, x:x+605]

        # Hide robot
        cv2.rectangle(img, (220, 210), (380, 345), 0, -1)

        return img

    def get_blocks(self):
        img = self.get_raw_image()
        img = self.preprocess(img)
        all_corners = find_blocks(img)

        blocks = []
        for i in range(0, len(all_corners)):
            block = Block(all_corners[i])
            cv2.circle(img, (block.x, block.y), 2, (0, 0, 255), -1)
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
