# FindTheSun.py
# This script interfaces with servos and a pi camera to track the sun
# across the sky. The script looks for the sun in an image, and then
# sends the appropriate signal to the servo to center the sun. If the
# sun is not found in an image, then a search algorithm that covers
# the entire sky is run.

# TODO
# -record more frames?
# -rewrite lost sun code?
# -measure fps!
from picamera import PiCamera
from skimage import measure
from imutils import contours
from shutil import copyfile
from adafruit_servokit import ServoKit
from imutils.video import VideoStream
import numpy as np
import RPi.GPIO as GPIO
import imutils
import time
import cv2
import math
import signal
import os

## GLOBALS ##
THRESHOLD = 120
BLUR_RADIUS = 11
MIN_ANGLE = 30
MAX_ANGLE = 80
X_RES = 640
Y_RES = 480
FPS_VID = 30
FPS = 11
WHITE = 255
THROTTLE_ZERO = -0.1
MAX_RADIUS = 75

X_CENTER = X_RES/2
Y_CENTER = Y_RES/2
CREATE_TIME = str(time.time())
MAX_THROTTLE = 0.3 + THROTTLE_ZERO

# mutable globals
j = 0
sign = 1

## FUNCTIONS ##
# signal handler for ctrl+c
def stop_servos(signum, sf):
    kit.continuous_servo[1].throttle = THROTTLE_ZERO
    end = time.perf_counter()
    print(f"stopping: {round(end-start,2)} seconds to process {j} frames")
    exit()

# main locate sun function
# returns the image and the countors found
def locate_sun():
    image = vs.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (BLUR_RADIUS,BLUR_RADIUS), 0)

    # threshold the image
    thresh = cv2.threshold(blurred, THRESHOLD, WHITE, cv2.THRESH_BINARY)[1]

    labels = measure.label(thresh, background=0, connectivity=2)
    mask = np.zeros(thresh.shape, dtype="uint8")
    numPixelMax=0

    for label in np.unique(labels):
        if label == 0:
            continue
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = WHITE
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
        labelMask[labels == label] = WHITE
        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if label == placement:
            mask = cv2.add(mask, labelMask)

    # find the contours in the mask, then sort them 
    # from left to right
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    return image, cnts

# logging function
# out: cv2.VideoWriter object
# image: image to be logged
# j: iteration of loop, decides whether to backup or save
# accept_data: marks whether frame is valid or not
def log_frames(out, image, accept_data, j):
    # demarcate valid frames
    output_string = str(time.time()) + ", " + str(accept_data) + "\n"
    f = open("./logs/valid_frames-" + CREATE_TIME + ".txt", "a")
    f.write(output_string)
    f.close()

    # save a frame to video every 15 seconds
    if j % (10 * FPS) == 0:
        print("writing image to video file...")
        out.write(image)
        # back up logs every 60 seconds
        if j % (60 * FPS) == 0:
            print("backing up logs...")
            copyfile("/home/pi/FindTheSunFinal/logs/valid_frames-" + CREATE_TIME + ".txt", "/home/pi/FindTheSunFinal/logs/backup-" + str(time.time()) + ".txt")
            # save a frame and backup video every 300 seconds (5 minutes)
            if j % (300 * FPS) == 0:
                print("saving frames...")
                cv2.imwrite("./frames/frame-" + str(time.time()) + ".bmp", image)
                print("backing up video...")
                copyfile("/home/pi/FindTheSunFinal/videos/processed-" + CREATE_TIME + ".avi", "/home/pi/FindTheSunFinal/videos/backup-" + str(time.time()) + ".avi")
    return j+1

## MAIN EXECUTION ##
if __name__ == '__main__':
    # change CWD to location of script
    os.chdir("/home/pi/FindTheSunFinal/")

    # set up servos
    kit = ServoKit(channels=16)
    # set the total range of servo in port 0 and 2 to 270
    kit.servo[0].actuation_range = 270
    kit.servo[2].actuation_range = 270
    # can tune the max and min pwm to max the servo go to desired angles
    kit.servo[0].set_pulse_width_range(500, 2500)
    kit.servo[2].set_pulse_width_range(500, 2500)
    kit.servo[0].angle = (MIN_ANGLE + MAX_ANGLE) / 2
    kit.servo[2].angle = (MIN_ANGLE + MAX_ANGLE) / 2
    time.sleep(0.3)
    kit.continuous_servo[1].throttle = 1 + THROTTLE_ZERO
    time.sleep(0.01)
    kit.continuous_servo[1].throttle = THROTTLE_ZERO

    # initialize PID values
    Ix = 0
    Iy = 0
    Ex_last = 0
    Ey_last = 0

    # initialize throttle and angle values
    angle_last = MAX_ANGLE/2
    throttle_last = THROTTLE_ZERO

    # set up video
    vs = VideoStream(usePiCamera=False, resolution=(X_RES,Y_RES)).start()
    time.sleep(0.3)
    out = cv2.VideoWriter("./videos/processed-" + CREATE_TIME + ".avi", cv2.VideoWriter_fourcc('M','J','P','G'), FPS_VID, (X_RES,Y_RES), isColor=True)

    # set up signal handler
    signal.signal(signal.SIGINT, stop_servos)
    signal.signal(signal.SIGTERM, stop_servos)

    # set up file
    f = open("./logs/valid_frames-" + CREATE_TIME + ".txt", "w+")
    f.write("header\n")
    f.close()

    start = time.perf_counter()
    # main loop
    while True:
        # get image and contours
        try:
            image, cnts = locate_sun()
            # if no countours, assume lost sun and start search protocol
            if cnts == []:
                print("LOST SUN :o")
                # set throttle to constant value
                if throttle_last >= THROTTLE_ZERO:
                    throttle_last = 0.2 + THROTTLE_ZERO
                else:
                    throttle_last = -0.2 + THROTTLE_ZERO
                kit.continuous_servo[1].throttle = throttle_last
                   
                image, cnts = locate_sun() 
                while cnts == []:
                    angle_lost = angle_last + (2 * sign)
                    angle_last = angle_lost
                    if angle_lost < MIN_ANGLE:
                        angle_lost = MIN_ANGLE
                        sign = -sign
                    elif angle_lost > MAX_ANGLE:
                        angle_lost = MAX_ANGLE
                        sign = -sign

                    print("sign: ", sign, "throttle: ", throttle_last, "angle: ", angle_lost)
                    kit.servo[0].angle = angle_lost
                    kit.servo[2].angle = angle_lost
                    
                    if abs(throttle_last) < 0.08 + THROTTLE_ZERO:
                        throttle_last = 0.10 + THROTTLE_ZERO
                        kit.continuous_servo[1].throttle = throttle_last

                    j = log_frames(out, image, 0, j) 
            
                    time.sleep(0.1)

                print("FOUND SUN")
                throttle_last = THROTTLE_ZERO
                kit.continuous_servo[1].throttle = throttle_last 
                    
            # loop over the contours
            cnts = contours.sort_contours(cnts)[0]
            for (i, c) in enumerate(cnts):
                ((cX, cY), radius) = cv2.minEnclosingCircle(c)
                # draw red circle of min enclosing circle of bright spot
                cv2.circle(image, (int(cX), int(cY)), int(radius), (0, 0, 255), 3)
                # draw green circle of acceptable radius
                cv2.circle(image, (int(X_CENTER), int(Y_CENTER)), MAX_RADIUS, (0, 255, 0), 3)

            # lets do some PID shiz
            Errorx = cX - X_CENTER
            Errory = cY - Y_CENTER

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
            Dx = (Ex_last)*Dvalx
            Dy = (Ey_last)*Dvaly
            Ex_last = Errorx
            Ey_last = Errory
            throttle_curr = Px + Ix + Dx + THROTTLE_ZERO
            PIDy = Py + Iy + Dy
            angle_curr = angle_last + PIDy

            if PIDy >= 0:
                sign = 1
            else:
                sign = -1
            
            # ensure angle is inbounds
            if angle_curr < MIN_ANGLE:
                angle_curr = MIN_ANGLE
            elif angle_curr > MAX_ANGLE:
                angle_curr = MAX_ANGLE

            # ensure throttle is inbounds
            if throttle_curr < -MAX_THROTTLE:
                throttle_curr = -MAX_THROTTLE
            elif throttle_curr > MAX_THROTTLE:
                throttle_curr = MAX_THROTTLE

            # set servos to appropriate throttle and angle
            kit.servo[0].angle = angle_curr
            kit.servo[2].angle = angle_curr + 5.0
            kit.continuous_servo[1].throttle = throttle_curr
                
            # update last angle + throttle
            angle_last = angle_curr
            throttle_last = throttle_curr
            
            print("P:", Py, "I:", Iy, "D:", Dy, "throttle:", throttle_curr, "angle:", angle_curr)
            
            # calculate distance from sun and decide whether it's close enough
            accept_data = 1
            distance = math.sqrt(Errorx ** 2 + Errory ** 2)
            if distance >= MAX_RADIUS:
                accept_data = 0
            
            j = log_frames(out, image, accept_data, j)
            time.sleep(0.00)
        # catch any exceptions log them and continue
        except Exception as e:
            print(e)
