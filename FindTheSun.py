from picamera import PiCamera
from skimage import measure
from imutils import contours
import imutils
from imutils.video import VideoStream
import numpy as np
import RPi.GPIO as GPIO
import time
from adafruit_servokit import ServoKit
import cv2
import math
import signal
import os

## GLOBALS ##

# set up the Servos (x servo (azimuth) into pin 11) (y servo into ___)
maxangle = 65
kit = ServoKit(channels=16)
kit.servo[0].actuation_range = 270 # set the total range of servo in port 0 to 60
kit.servo[0].set_pulse_width_range(500, 2500) # can tune the max and min pwm to max the servo go to desired angles
kit.servo[2].actuation_range = 270 # set the total range of servo in port 0 to 60
kit.servo[2].set_pulse_width_range(500, 2500) # can tune the max and min pwm to max the servo go to desired angles
kit.servo[0].angle = maxangle/2
kit.servo[2].angle = maxangle/2
time.sleep(0.3)
kit.continuous_servo[1].throttle = 1
time.sleep(0.01)
kit.continuous_servo[1].throttle = 0

# global variables
os.chdir("/home/pi/Documents/PythonCodes/")
Ix = 0
Iy = 0
Errorxlast = 0
Errorylast = 0
anglelast = maxangle/2
throttlelast = 0
xRes = 640
yRes = 480
xCenter = xRes/2
yCenter = yRes/2
fps = 30
THRESHOLD = 120
SIGN = 1
BLUR_RADIUS = 11
l = 0
throttle1 = 0
white = 255
maxthrottle = 0.3
backup = 0
# set up video
vs = VideoStream(usePiCamera=False, resolution=(xRes,yRes)).start()
time.sleep(0.3)
out = cv2.VideoWriter("./videos/processed" + str(time.time()) + ".avi", cv2.VideoWriter_fourcc('M','J','P','G'), fps, (xRes,yRes), isColor=True)

## FUNCTIONS ##
# signal handler for ctrl+c
def stop_servos(signum, sf):
    kit.continuous_servo[1].throttle = 0
    end = time.perf_counter()
    print(f"stopping: {round(end-start,2)} seconds to process {j} frames")
    exit()

def locate_sun():
    image = vs.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (BLUR_RADIUS,BLUR_RADIUS), 0)

    # threshold the image
    thresh = cv2.threshold(blurred, THRESHOLD, white, cv2.THRESH_BINARY)[1]

    labels = measure.label(thresh, background=0, connectivity=2)
    mask = np.zeros(thresh.shape, dtype="uint8")
    numPixelMax=0

    for label in np.unique(labels):
        if label == 0:
            continue
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = white
        numPixels = cv2.countNonZero(labelMask)
        if numPixels > numPixelMax:
            placement = label
            numPixelMax = numPixels

    # loop over the unique components
    for label in np.unique(labels):
        # if this is the background label, ignore it
        if label == 0:
            continue
        # otherwise, construct the label mask and count the
        # number of pixels 
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = white
        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if label == placement:
            mask = cv2.add(mask, labelMask)


    # find the contours in the mask, then sort them from left to
    # right
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return image, cnts


## MAIN LOOP ##
signal.signal(signal.SIGINT, stop_servos)
j = 0
create_time = str(time.time())
f = open("./AreWeLookingAtTheSun/valid_frames" + create_time + ".txt", "w+") #AreWeLookingAtTheSun
f.write("header\n")
f.close()

start = time.perf_counter()
while True:
    image, cnts = locate_sun()
    
    # check if empty
    if cnts == []:
        print("LOST SUN :o")
        if throttlelast >= 0:
            throttlelast = 0.2
            kit.continuous_servo[1].throttle = 0.2
        else:
            throttlelast = -0.2
            kit.continuous_servo[1].throttle = -0.2
            
        while True:
            image, cnts = locate_sun()
            if cnts != []:
                print("FOUND SUN")
                kit.continuous_servo[1].throttle = 0
                throttlelast = 0
                break
            anglelost = anglelast + (4*SIGN)
            anglelast = anglelost
            if anglelost < 0:
                anglelost = 0
                SIGN = -SIGN
            elif anglelost > maxangle:
                anglelost = maxangle
                SIGN = -SIGN
            else:
                anglelost = anglelost
            print("anglelost", anglelost, "SIGN", SIGN, "THROTTLE", throttlelast)
            print("angle: " + str(anglelost))
            kit.servo[0].angle = anglelost
            kit.servo[2].angle = anglelost
            
            if abs(throttlelast) < 0.08:
                throttlelast = 0.10
                kit.continuous_servo[1].throttle = throttlelast

            output_string = str(time.time()) + ", 0\n"
            f = open("./AreWeLookingAtTheSun/valid_frames" + create_time + ".txt", "a")
            f.write(output_string)
            f.close()

            # write output image to avi file
            out.write(image)
            # save a frame every 5 minutes (3240 frames)
            if j % 3240 == 0:
                print("saving frames...")
                cv2.imwrite("./frames/frame.bmp", image)
            # every 100 frames backup valid_frames file
            if j % 100 == 0:
                print("backing up")
                os.system("cp " + "./AreWeLookingAtTheSun/valid_frames" + create_time + ".txt " + "./AreWeLookingAtTheSun/backup" + str(backup) + "-" + str(time.time()) + ".txt")
                backup += 1            
    
            j += 1
            time.sleep(0.2)
            
    cnts = contours.sort_contours(cnts)[0]

    # loop over the contours
    for (i, c) in enumerate(cnts):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        cv2.circle(image, (int(cX), int(cY)), int(radius),
            (0, 0, 255), 3)
        cv2.circle(image, (int(xCenter), int(yCenter)), 75, (0, 255, 0), 3)

    # lets do some PID shiz
    Errorx = cX-xCenter
    Errory = cY-yCenter

    Pvalx = 0.0012
    Pvaly = 0.015
    Px = Pvalx * Errorx
    Py = Pvaly * Errory   
    Ivalx = 0.00015
    Ivaly = 0.0001
    Ix = Ix + ((Errorx)*Ivalx)
    Iy = Iy + ((Errory)*Ivaly)
    Dvalx = 0.0026
    Dvaly = 0.0002
    Dx = (Errorxlast)*Dvalx
    Dy = (Errorylast)*Dvaly
    Errorxlast = Errorx
    Errorylast = Errory
    throttle1 = Px + Ix + Dx
    PIDy = Py + Iy + Dy
    angle0 = anglelast + PIDy

    if PIDy >= 0:
        SIGN = 1
    else:
        SIGN = -1
    if angle0 < 0:
        angle0 = 0
    elif angle0 > maxangle:
        angle0 = maxangle
    else:
        angle0 = angle0
    if throttle1 < -maxthrottle:
        throttle1 = -maxthrottle
    elif throttle1 > maxthrottle:
        throttle1 = maxthrottle
    else:
        throttle1 = throttle1
    if j < 5:
        kit.servo[0].angle = maxangle/2
        kit.servo[2].angle = maxangle/2

    else:
        print("angle: " + str(angle0))
        kit.servo[0].angle = angle0
        kit.servo[2].angle = angle0+5.0
        kit.continuous_servo[1].throttle = throttle1     
    anglelast = angle0
    throttlelast = throttle1
    
    print("P: ", Py, "I: ", Iy, "D: ", Dy, "throttle: ", throttle1)
    
    radiusnewone = Errorx ** 2 + Errory ** 2
    if radiusnewone <= 5625:
        output_string = str(time.time()) + ", 1\n"
    else:
        output_string = str(time.time()) + ", 0\n"
    f = open("./AreWeLookingAtTheSun/valid_frames" + create_time + ".txt", "a")
    f.write(output_string)
    f.close()
    
    # write output image to avi file
    out.write(image)
    # save a frame every 5 minutes (3240 frames)
    if j % 3240 == 0:
        print("saving frames...")
        cv2.imwrite("./frames/frame.bmp", image)
    # every 100 frames backup valid_frames file
    if j % 100 == 0:
        print("backing up")
        os.system("cp " + "./AreWeLookingAtTheSun/valid_frames" + create_time + ".txt " + "./AreWeLookingAtTheSun/backup" + str(time.time()) + ".txt")

    j += 1
    time.sleep(0.00)

kit.continuous_servo[1].throttle = 0  
f.close()  
