import cv2
# import cv2.
import numpy as np

def main():
    # win = cv2.namedWindow("hi", cv2.WINDOW_FULLSCREEN)
    dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

    im = np.ones((256, 256, 3), dtype=np.uint8)
    # im *= 255

    cv2.aruco.drawMarker(dict, 0, 160, im, 6)
    print(im)
    cv2.imshow("hi", im)
    cv2.waitKey(0)
if __name__ == "__main__":
    main()