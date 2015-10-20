#!/usr/bin/env python

import rospy
from CrustCrawler import CrustCrawler
from Camera import Camera

if __name__ == "__main__":
    rospy.init_node("au_dynamixel_test_node")

    crustCrawler = CrustCrawler()
    camera = Camera()

    right = []
    left = []

    crustCrawler.reset()
    blocks = camera.get_blocks()

    #while True:
    #    crustCrawler.reset()
    #    blocks = camera.get_blocks()

    #    if len(blocks) == 0:
    #        break

        #for block in blocks:
        #    if len(right) == 0 or block.same_color(right[0].color):
        #        crustCrawler.place_block_right(block, len(right))
        #        right.append(block)
        #    else:
        #        crustCrawler.place_block_left(block, len(left))
        #        left.append(block)
