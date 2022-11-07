# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import glob

# The purpose of this program is to:
# 1. take (5-10) images of a 6x7 black and white chess board and save each to the direction in which this file is stored
# 2. calculate the distortion and matrix coeffiecients of the camera to use for demo2



# ======================= MAIN =======================
if __name__ == "__main__":

    cap = cv.VideoCapture(0)
    cv.namedWindow("Live Feed")

    imagesOrNaw = input("Do you want to take new images? (y or n)")
    print ("Press b to take an image, press q to quit.")
    retry = "n"
    i = 0
    
    if not cap.isOpened():
        print("Can't open camera.. RIP")
        exit()
        
    while True:
        ret, frame = cap.read() # read a frame
        if not ret:
            print("No frame detected.. RIP")
            break
        
        if (imagesOrNaw=="n"): break

        if not ret:
            print("Couldn't capture frame")
            break

        cv.imshow("Live Feed", frame)

        # press q to exit
        if (cv.waitKey(1) == ord("q")):
            break

        # press b to take new image
        elif (cv.waitKey(1) == ord("b")):
            print("Image taken")
            fileName = str(i+1) + ".jpg"
            cv.imwrite(fileName, frame)
            print("Saved " + fileName)
            i += 1
            

    cap.release() #video end
    cv.destroyAllWindows()
        
        
    calibrate = input("Calibration? (y or n) ")
    if (calibrate == "y"):
        # termination criteria
        criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*7,3), np.float32)
        objp[:,:2] = np.mgrid[0:7,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d point in real world space
        imgpoints = [] # 2d points in image plane.
        images = glob.glob('*.jpg')
        flag = False
        for fname in images:
            print(fname)
            img = cv.imread(fname)
            gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
            # Find the chess board corners
            ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
            print(ret)
            # If found, add object points, image points (after refining them)
            if ret == True:
                flag = True
                print("X")
                objpoints.append(objp)
                corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
                imgpoints.append(corners2)
                # Draw and display the corners
                cv.drawChessboardCorners(img, (7,6), corners2, ret)
                
                cv.imshow('Chessbord Img', img)
                # press q to exit
                if (cv.waitKey(1) == ord("q")):
                    break
        cv.destroyAllWindows()

        if (flag):
            ret, mtx, dist, rvec, tvec = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
            print("rvec: ", rvec)
            print("tvec: ", tvec)
