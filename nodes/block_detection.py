import cv2
import numpy as np


def draw_contours(contours, hierarchy, image, parent=-1, level=0):
    colors = [
        (0,0,255),
        (255,0,0),
        (0,255,0),
    ]

    for i in range(0, len(contours)):
        # iterate parents children only
        if hierarchy[i, 3] != parent:
            continue

        cv2.drawContours(image, [contours[i]], -1, colors[level], 2)

        if hierarchy[i, 2] != -1:
            draw_contours(contours, hierarchy, image, i, level+1)


# Returns a block if a contour qualifies
def block_from_contour(contour):
    epsilon = 0.1 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    rect = cv2.minAreaRect(approx)
    block = cv2.cv.BoxPoints(rect)
    block = np.int0(block)
    area = cv2.contourArea(approx)
    width = np.linalg.norm(block[1] - block[0])
    height = np.linalg.norm(block[1] - block[2])
    wh_ratio = width/height if width/height <= 1 else height/width

    if 800 < area < 2500 and 10 < epsilon < 22 and wh_ratio > 0.6:
        return block
    else:
        return []


# recursivly find blocks from contour tree.
def blocks_from_contour_tree(contours, hierarchy, parent=-1):
    blocks = []

    # iterate children of parent
    for i in range(0, len(contours)):
        if hierarchy[i, 3] != parent:
            continue

        children_blocks = []

        if hierarchy[i, 2] != -1:
            children_blocks = blocks_from_contour_tree(contours, hierarchy, i)

        if len(children_blocks) > 0:
                blocks.extend(children_blocks)
        else:
            block = block_from_contour(contours[i])
            if len(block) > 0:
                blocks.append(block)

    return blocks


def find_blocks(image):
    img = image

    # Smoothen image
    img = cv2.adaptiveBilateralFilter(img, (9, 9), 100)

    # Sharpen image with laplacian
    lp = cv2.Laplacian(img, cv2.CV_8U)

    ret, lp = cv2.threshold(lp, 10, 255, cv2.THRESH_BINARY)
    img += 12*lp

    # Detect edges
    img = cv2.Canny(img, 35, 100)

    # Dilate edges
    kernel = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    img = cv2.dilate(img, kernel, iterations=3)

    # Close edges
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)

    # Erode edges
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
    img = cv2.erode(img, kernel, iterations=1)

    # Find contours
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Draw contours
    #draw_contours(contours, hierarchy[0], image)

    # Find blocks from contours
    blocks = blocks_from_contour_tree(contours, hierarchy[0])

    return blocks
