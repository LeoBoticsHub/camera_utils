import cv2
import imutils
import numpy as np

# Intel RealSense D415 Federico
# yellowBallLower = (55, 60, 6)
# yellowBallUpper = (85, 255, 255)


def get_ball_center(color_frame, yellow_ball_lower, yellow_ball_upper):
    blurred = cv2.GaussianBlur(color_frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv2.inRange(hsv, yellow_ball_lower, yellow_ball_upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    x, y, radius = None, None, None
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        if cv2.contourArea(c) > 80:
            ((x, y), radius) = cv2.minEnclosingCircle(c)

    return [x, y, radius]


def colorMask(color_frame, yellow_ball_lower, yellow_ball_upper):
    mask = np.zeros((1080, 1920), dtype="uint8")

    # center_and_radius = getBallCenter(color_frame)
    x, y, radius = get_ball_center(color_frame, yellow_ball_lower, yellow_ball_upper)
    try:
        cv2.circle(mask, (int(x), int(y)), int(radius), (255, 255, 255), -1)
    except TypeError:
        mask = np.ones((1080, 1920), dtype="uint8")*255

    # mask[mask > 254] = True
    mask = mask > 254

    return mask
