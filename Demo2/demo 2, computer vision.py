# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import glob
import smbus

import board
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04


def writeNumber(value):

    #bus.write_byte(address, value)
    #bus.write_byte_data(address, 0, value)
    try:
       bus.write_i2c_block_data(address, 0, value)
    except IOError:

        print ("I2C Error")

    return -1

def readNumber():

    #number = bus.read_byte(address)
   
    #number = bus.read_byte_data(address, 0)
    number = ["0"]

    try:
        number = bus.read_i2c_block_data(address,0, datasize)

    except IOError:
        print ("I2C Error")

    return number


# ======================= MAIN =======================
if __name__ == "__main__":
   
    #TODO: calibrate camera to get matCoef and distCoef
    distCoef = np.array([8.77895954e-02, 4.07658687e+00, -1.12123971e-02, 1.35107554e-02, -3.18493348e+01], dtype=np.float32)
    matCoef = np.array([[7.9898851731e+02, 0., 3.4328356227e+02], [0., 8.0056742045e+02, 1.992733627e+02], [0., 0., 1.]], dtype=np.float32)
       
    num = [0,0,0,0] #for arduino, use writeNumber
    #BYTES TO SEND:
    # 1st = 0 or 1 (neg or pos angle)
    # 2nd = first 2 digits of angle
    # 3rd = next 2 digits of angle
    # 4th = distance in cm
   
    # allow the camera to warmup
    time.sleep(0.1)

    # create aruco dictionary of 250, 6x6 ID's
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    aruco_params = aruco.DetectorParameters_create()
   
    ready = input("Ready to start video? Input anything when ready:")
   
    cap = cv.VideoCapture(0) # start video
    if not cap.isOpened():
        print("Can't open camera.. RIP")
        exit()
       
    while True:
        #writeNumber("hi") #delete later
        ret, frame = cap.read() # read a frame
        if not ret:
            print("No frame detected.. RIP")
            break

        imageG = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # convert each frame to grayscale
        (corners, ids, rejected) = aruco.detectMarkers(imageG, aruco_dict, parameters=aruco_params) # detect aruco markers and get data
        cornersTemp = corners
       
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH) # get width and height of video frame
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)

        if (ids != None): # if markers detected

            aruco.drawDetectedMarkers(frame, corners)
            for (markerCorner, markerID) in zip(corners, ids): # convert corners numpy arry into tuple
                corners = markerCorner.reshape((4,2))
                (lt, rt, rb, lb) = corners # left top, right top, right bot, left bot corners respectively (pixel coords)

            # GET PIXEL COORDS AND CALC ANGLE
            # camera (0,0) is the top left
            # the pixel coords of center of detected marker is the midpoint between lt and rb:
            pixelCenter = [-1, -1]
            pixelCenter[0] = (lt[0] + rb[0])/2  #pixelu
            pixelCenter[1] = (lt[1] + rb[1])/2  #pixelv
           
            # angle is calculated by using the FOV of the camera (57 degrees) over 2, multiplied by the distance between the X coord of the center
            # of the detected marker over the pixel coords of the center of the marker. - to the right + to the left
            angleDeg = 57/2 * (width/2 - pixelCenter[0]) / (width/2)
           
   
            # TODO: GET RVEC AND TVEC, AND CALC DISTANCE
            rvec, tvec, _ = aruco.estimatePoseSingleMarkers(cornersTemp, 5, matCoef, distCoef)
           
            print("distance?: ", int(round(tvec[0][0][2]/1.35)))
            # TODO: convert distance to CM
           
            # SEND ANGLE AND DIST TO ARDUINO
            if (angleDeg < 0):
                num[0] = 1 #angle is neg
                writeNumber(num)
                angleDeg *= -1
            else:
                num[0] = 0 #angle is pos
                writeNumber(num)

            angleRad = "%.4f" % round(math.radians(angleDeg), 4) #radians for arduino .4578
            angleRadSECOND = int(angleRad[2]+angleRad[3])
            angleRadTHIRD = int(angleRad[4]+angleRad[5])
            num[1] = angleRadSECOND
            writeNumber(num)
            num[2] = angleRadTHIRD
            writeNumber(num)
            num[3] = int(round(tvec[0][0][2]/1.35))
            print(angleRad)
           
            writeNumber(num) #this was changed

            #TODO: send distanceCM
           
            #TODO: add handshake between arduino for all sends

        cv.imshow("Video", frame) # display video
        # press q to exit
        if cv.waitKey(1) == ord("q"):
            break

    cap.release() # video end
    cv.destroyAllWindows()
