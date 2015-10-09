#!/usr/bin/env python

import rospy
from CrustCrawler import CrustCrawler
from Camera import Camera

if __name__ == "__main__":
    rospy.init_node("au_dynamixel_test_node")

    crustCrawler = CrustCrawler()
    camera = Camera()

    crustCrawler.reset()

    camera.get_blocks()

    crustCrawler.open_gripper()
    crustCrawler.close_gripper()
