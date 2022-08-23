import numpy as np
import cv2

def getPointsPositions(image, display_img = False):

    gray_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    circles = cv2.HoughCircles(gray_img, cv2.HOUGH_GRADIENT, 2, 80, param1=24, 
                               param2=13, minRadius=0, maxRadius=15)

    print(circles)
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        
        if display_img:
            out_img = image.copy()
            for (x, y, r) in circles:
                cv2.circle(out_img, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(out_img, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

            cv2.namedWindow("Found Circles", cv2.WINDOW_NORMAL)
            cv2.imshow("Found Circles", out_img)
            cv2.waitKey(0)
    
    return circles
