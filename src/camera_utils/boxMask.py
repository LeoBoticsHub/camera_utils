# import the necessary packages
import cv2

def getBoxMask(color_frame):
    # box_lower_hsv_color = (10, 25, 200)
    # box_upper_hsv_color = (20, 100, 255)
    box_lower_hsv_color = (10, 50, 70)
    box_upper_hsv_color = (30, 150, 190)

    hsv = cv2.cvtColor(color_frame, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, box_lower_hsv_color, box_upper_hsv_color)
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)

    return mask
