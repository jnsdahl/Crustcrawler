#!/usr/bin/env python

import rospy
from CrustCrawler import CrustCrawler
from Camera import Camera

if __name__ == "__main__":
    rospy.init_node("au_dynamixel_test_node")

    crustCrawler = CrustCrawler()
    camera = Camera()

    crustCrawler.reset()

    blocks = camera.get_blocks()

    for block in blocks:
        crustCrawler.move_to(block.x, block.y, block.z, block.theta)
        crustCrawler.open_gripper()
        crustCrawler.move_to(block.x, block.y, block.z - 12, block.theta)
        crustCrawler.close_gripper()
        crustCrawler.reset()
