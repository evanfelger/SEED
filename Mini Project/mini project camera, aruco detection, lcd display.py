# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2 as cv
from cv2 import aruco
import numpy as np
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

        print (" I2C Error")
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
        print (" I2C Error")
        # Print LCD message
        lcd.message = "I2C Error"

    return number



# ======================= MAIN =======================
if __name__ == "__main__":

    

    # allow the camera to warmup
    time.sleep(0.1)

    print("Turning on the camera and calibrating...")

    try:
        pass
    except:
        print("Failed to calibrate")
        exit()
    
    print("Calibration complete.")

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
        cv.imshow("Video", imageG) # display video
        (corners, ids, rejected) = aruco.detectMarkers(imageG, aruco_dict, parameters=aruco_params) # detect aruco markers and get data
        
        width = cap.get(cv.CAP_PROP_FRAME_WIDTH) # get width and height of video frame
        height = cap.get(cv.CAP_PROP_FRAME_HEIGHT)



        num = ["0"]
        
    
        
        if (ids != None): # if markers detected
            # get corner coords of marker to detemine which quadrant its in
            for (markerCorner, markerID) in zip(corners, ids): # convert corners numpy arry into tuple
                corners = markerCorner.reshape((4,2))
                (lt, rt, rb, lb) =  corners # left top, right top, right bot, left bot respectively
    
            if ((lt[0]>width/2 and lb[0]>width/2) and (lb[1]<height/2 and rb[1]<height/2)): #top right (QI)
                #print("Marker is in Quadrant 1")
                datasize = 1
                num[0] = 1
               # num = num.to_bytes(2,'little')
                writeNumber(num)
                lcd.message = "1"
            elif ((lt[0]<width/2 and lb[0]<width/2) and (lb[1]<height/2 and rb[1]<height/2)): #top left (QII)
                #print("Marker is in Quadrant 2")
                datasize = 1
                num[0] = 2
              #  num = num.to_bytes(2,'little')
                writeNumber(num)
                lcd.message = "2"
            elif ((lt[0]<width/2 and lb[0]<width/2) and (lb[1]>height/2 and rb[1]>height/2)): #bot left (QIII)
                #print("Marker is in Quadrant 3")
                datasize = 1
                num[0] = 3
             #   num = num.to_bytes(2,'little')
                writeNumber(num)
                lcd.message = "3"
            elif ((lt[0]>width/2 and lb[0]>width/2) and (lb[1]>height/2 and rb[1]>height/2)): #bot right (QIV)
                #print("Marker is in Quadrant 4")
                datasize = 1
                num[0] = 4
              #  num = num.to_bytes(2,'little')
                writeNumber(num)
                lcd.message = "4"

        # press q to exit
        if cv.waitKey(1) == ord("q"):
            break

    cap.release() # video end
    cv.destroyAllWindows()
