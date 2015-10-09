import numpy as np
from math import atan2, cos, sin, pi

class Block:
    def __init__(self, corners):
        self.x, self.y, self.z = self.img2base_transform(
            corners[2][0] - (corners[0][0] - corners[2][0]) / 2,
            corners[3][1] - (corners[1][1] - corners[3][1]) / 2
        )

        self.theta = self.find_orientation(corners)

    # Transforms a point on the image to the robot base frame
    def img2base_transform(self, x, y):
        # Transform point on image to table frame
        point = np.array([ x / 8.765, y / 8.765, -5, 1])
        point = point[:, None]

        # Transform point in table frame to robot base frame
        tx = -34
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
    def find_orientation(self, block_corners):
        min_theta = 0

        for i in range(0, 2):
            corner1 = self.img2base_transform(block_corners[i][0], block_corners[i][1])
            corner2 = self.img2base_transform(block_corners[i+1][0], block_corners[i+1][1])

            dx = corner1[0] - corner2[0]
            dy = corner1[1] - corner2[1]

            theta = atan2(dy, dx)

            if theta < min_theta or i == 0:
                min_theta = theta

        return min_theta
