#!/usr/bin/env python

import rospy
import cv2
from CrustCrawler import CrustCrawler
from Camera import Camera

if __name__ == "__main__":
    rospy.init_node("au_dynamixel_test_node")

    crustCrawler = CrustCrawler()
    camera = Camera()

    right = []
    left = []

    while True:
        crustCrawler.reset()
        blocks, img = camera.get_blocks()
        cv2.imshow('Camera', img), cv2.waitKey(40)

        for block in blocks:
            if len(right) == 0 or block.same_color(right[0].color):
                crustCrawler.place_block_right(block, len(right))
                right.append(block)
            elif len(left) == 0 or block.same_color(left[0].color):
                crustCrawler.place_block_left(block, len(left))
                left.append(block)
