import numpy as np
from math import atan2, cos, sin, pi

class Block:
    def __init__(self, corners):
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

        self.x, self.y, self.z = self.img2base_transform(xc, yc)
        self.thetas = self.find_orientations(corners)

    # Transforms a point on the image to the robot base frame
    def img2base_transform(self, x, y):
        # Transform point on image to table frame
        point = np.array([ x / 8.765, y / 8.765, -10, 1])
        point = point[:, None]

        # Transform point in table frame to robot base frame
        tx = -33
        ty = 35
        tz = 0
        thetax = pi
        thetaz = pi / 2

        A = np.matrix([
            [1, 0,           0,            tx],
            [0, cos(thetax), -sin(thetax), ty],
            [0, sin(thetax), cos(thetax),  tz],
            [0, 0,           0,            1 ]
        ])

        B = np.matrix([
            [cos(thetaz),  sin(thetaz), 0, 0],
            [-sin(thetaz), cos(thetaz), 0, 0],
            [0,            0,           1, 0],
            [0,            0,           0, 1]
        ])

        point = B.dot(A.dot(point))

        return point[0], point[1], point[2]

    # Find orientation of a block
    def find_orientations(self, block_corners):
        thetas = []

        for i in range(0, 2):
            corner1 = self.img2base_transform(block_corners[i][0], block_corners[i][1])
            corner2 = self.img2base_transform(block_corners[i+1][0], block_corners[i+1][1])

            dx = corner2[0] - corner1[0]
            dy = corner2[1] - corner1[1]

            theta = atan2(dy, dx)

            thetas.append(theta)

        return thetas
