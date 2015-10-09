#!/usr/bin/env python

import cv2
from Camera import Camera

cam = Camera()

cam.get_blocks(True)

cv2.waitKey(0)