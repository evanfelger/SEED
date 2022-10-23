# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
from cv2 import aruco
import numpy as np
import math
import smbus

import board
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
# for RPI version 1, use “bus = smbus.SMBus(0)”
bus = smbus.SMBus(1)
# Modify this if you have a different sized Character LCD
lcd_columns = 16
lcd_rows = 2
# Initialise I2C bus.
i2c = board.I2C() # uses board.SCL and board.SDA
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()
# Set LCD color to purple
lcd.color = [50, 0, 50]

# This is the address we setup in the Arduino Program
address = 0x04

def writeNumber(value):

    #bus.write_byte(address, value)
    #bus.write_byte_data(address, 0, value)
    try:

        bus.write_i2c_block_data(address, 0, value)
    except IOError:

        print ("I2C Error")
        # Print LCD message
        lcd.message = "I2C Error"

    return -1

def readNumber():

    #number = bus.read_byte(address)
    #number = bus.read_byte_data(address, 0)
    number = [0]

    try:
        number = bus.read_i2c_block_data(address,0, datasize)

    except IOError:
        print ("I2C Error")
        # Print LCD message
        lcd.message = "I2C Error"

    return number


# ======================= MAIN =======================
if __name__ == "__main__":


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
        ret, frame = cap.read() # read a frame
        if not ret:
            print("No frame detected.. RIP")
            break

        imageG = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) # convert each frame to grayscale
        (corners, ids, rejected) = aruco.detectMarkers(imageG, aruco_dict, parameters=aruco_params) # detect aruco markers and get data
        
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH) # get width and height of video frame
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)

        
        angleDeg = 0
        angleRad = 0
        num = ["0"]

        if (ids != None): # if markers detected

            aruco.drawDetectedMarkers(frame, corners)
            for (markerCorner, markerID) in zip(corners, ids): # convert corners numpy arry into tuple
                corners = markerCorner.reshape((4,2))
                (lt, rt, rb, lb) = corners # left top, right top, right bot, left bot corners respectively (pixel coords)

            lcd.message = "Marker Detected!" #row 1

            # GET PIXEL COORDS
            # camera (0,0) is the top left
            # the pixel coords of center of detected marker is the midpoint between lt and rb:
            pixelCenter = [-1, -1]
            pixelCenter[0] = (lt[0] + rb[0])/2  #pixelu
            pixelCenter[1] = (lt[1] + rb[1])/2  #pixelv
            
            # angle is calculated by using the FOV of the camera (57 degrees) over 2, multiplied by the distance between the X coord of the center
            # of the detected marker over the pixel coords of the center of the marker
            angleDeg = round((57/2 * (width/2 - pixelCenter[0]) / (width/2)), 2) #rounded to 2 decimal points
            angleRad = round(math.degrees(angleDeg), 2) #radians for arduino
            
            lcd.message = "\nAngle: " + str(angleDeg) #row 2

        
        else:   
            lcd.clear()
            lcd.message = "No Marker Found."
            time.sleep(.5)

        #cv.imshow("Video", frame) # display video
        # press q to exit
        if cv.waitKey(1) == ord("q"):
            break

    cap.release() # video end
    lcd.clear()
    cv.destroyAllWindows()
