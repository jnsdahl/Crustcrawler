import numpy as np
from math import atan2, cos, sin, pi
import cv2

class Block:
    def __init__(self, corners, img):
        x1 = corners[0][0]
        x2 = corners[1][0]
        x3 = corners[2][0]
        x4 = corners[3][0]

        y1 = corners[0][1]
        y2 = corners[1][1]
        y3 = corners[2][1]
        y4 = corners[3][1]

        a1 = (y3-y1)/float(x3-x1)
        a2 = (y4-y2)/float(x4-x2)

        b1 = y1 - a1*x1
        b2 = y2 - a2*x2

        xc = (b2-b1)/float(a1-a2)
        yc = a1*xc+b1

        cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        color = [0, 0, 0]
        c = 0

        for y in range(int(yc)-5, int(yc)+5):
            for x in range(int(xc)-5, int(xc)+5):
                c += 1
                color[0] += img[y, x][0]
                color[1] += img[y, x][1]
                color[2] += img[y, x][2]

        color[0] = color[0]/c
        color[1] = color[1]/c
        color[2] = color[2]/c

        self.color = color
        self.corners = corners
        self.x, self.y, self.z = self.img2base_transform(xc, yc)

    def same_color(self, color):
        th = 70
        return abs(self.color[0] - color[0]) < th and abs(self.color[1] - color[1]) < th and abs(self.color[2] - color[2]) < th

    # Transforms a point on the image to the robot base frame
    def img2base_transform(self, x, y):
        # Transform point on image to table frame
        point = np.array([ x / 8.765, y / 8.765, -10, 1])
        point = point[:, None]

        # Transform point in table frame to robot base frame
        tx = -33.5
        ty = 35.5
        tz = 0
        thetax = pi
        thetaz = pi / 2

        A = np.matrix([
            [1, 0,            0,            tx],
            [0, cos(thetax),  sin(thetax),  ty],
            [0, -sin(thetax), cos(thetax),  tz],
            [0, 0,            0,            1 ]
        ])

        B = np.matrix([
            [cos(thetaz),  sin(thetaz), 0, 0],
            [-sin(thetaz), cos(thetaz), 0, 0],
            [0,            0,           1, 0],
            [0,            0,           0, 1]
        ])

        point = B.dot(A.dot(point))

        return point[0], point[1], point[2]

    def img2joint1_transform(self, x, y, q1):
        x, y, z = self.img2base_transform(x, y)

        # Transform point on image to table frame
        point = np.array([x, y, z, 1])
        point = point[:, None]

        # Transform point in table frame to robot base frame
        A = np.matrix([
            [cos(q1),  sin(q1), 0, 0],
            [-sin(q1), cos(q1), 0, 0],
            [0,            0,   1, 0],
            [0,            0,   0, 1]
        ])

        point = A.dot(point)

        return point[0], point[1], point[2]

    # Find orientation of a block
    def find_orientation(self, q1):
        min_theta = 0

        for i in range(0, 2):
            corner1 = self.img2joint1_transform(self.corners[i][0], self.corners[i][1], q1)
            corner2 = self.img2joint1_transform(self.corners[i+1][0], self.corners[i+1][1], q1)

            dx = corner2[0] - corner1[0]
            dy = corner2[1] - corner1[1]

            theta = atan2(dy, dx)

            if abs(theta) < abs(min_theta) or i == 0:
                min_theta = theta

        return min_theta
