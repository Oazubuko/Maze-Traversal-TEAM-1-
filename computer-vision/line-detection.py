import cv2
from typing import List
from utils.customtypes import Image, Line
import numpy as np

def identify_tape(img: Image) -> Image:
    '''Finds all pixels in the original image that are white tape'''
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    white_pixels = cv2.inRange(hsv, (0, 0, 215), (180, 20, 255), cv2.THRESH_BINARY)
    
    return white_pixels

def identify_tape_adaptive(img: Image) -> Image:
    gray_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 0)

def open_and_close_shapes(img: Image) -> Image:
    '''
    Removes noisy pixels found in a binary image by closing holes morphological
    operations (see https://www.wikiwand.com/en/Mathematical_morphology)
    '''
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    # Destroy small (noisy) holes
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)

    return img

def detect_lines(gray_img: Image) -> List[Line]:
    '''
    Uses a Hough Transform to detect lines in a grayscale image. 

    The Hough transform works by
        - Applying edge detection, which returns a list
          of (x, y) pixels that are edges
        - Create a grid of possible lines specified in polar coordinates (p, theta)
        - For each edge pixel (x, y), determine which line it's most likely to contribute
          to and increment the corresponding grid value
        - Threshold the grid to find the most likely lines
    '''
    edges = cv2.Canny(gray_img, 50, 200)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 30, minLineLength=50, maxLineGap=25)
    
    return lines

def draw_lines(img: Image, lines: List[Line]) -> Image:
    '''Draws a list of lines on an image'''
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

    white_tape = identify_tape(img)
    white_tape = open_and_close_shapes(white_tape)

    lines = detect_lines(white_tape)
    img = draw_lines(img, lines)
    

    cv2.imshow('White Detection', white_tape)
    cv2.imshow('Line Detection', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()