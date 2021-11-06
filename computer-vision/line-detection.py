import cv2
from typing import List
from utils.customtypes import Image, Line
import numpy as np

def detect_maze_lines(img: Image) -> List[Line]:
    ''''''

    
    return img

def detect_lines(img: Image) -> List[Line]:
    '''
    Uses a Hough Transform to detect lines in an image. 
    The Hough transform works.
    '''
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, minLineLength=50, maxLineGap=25)
    
    return lines

def draw_lines(img: Image, lines: List[Line]) -> Image:
    '''Draws the lines on an image'''
    if lines is None:
        return img

    LINE_COLOR = (255, 0, 0)

    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), LINE_COLOR, 3)

    return img



if __name__ == "__main__":
    TEST_IMAGE = 'test-images/simple-maze.jpg'
    img = cv2.imread(TEST_IMAGE)

    lines = detect_lines(img)
    img = draw_lines(img, lines)

    cv2.imshow('Line Detection', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()