# FindTheSun.py
# This script interfaces with servos and a pi camera to track the sun
# across the sky. The script looks for the sun in an image, and then
# sends the appropriate signal to the servo to center the sun. If the
# sun is not found in an image, then a search algorithm that covers
# the entire sky is run.

# TODO
# - rethink starting directory?
# - rewrite logging function to file?
# - catch any errors the camera might throw
# - adjust backup frequency
# - use shutil for copying??
from venv import create
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
THRESHOLD = 120
SIGN = 1
BLUR_RADIUS = 11
MAX_ANGLE = 65
X_RES = 640
Y_RES = 480
FPS = 30
WHITE = 255
THROTTLE_ZERO = -0.1
MAX_RADIUS = 75

X_CENTER = X_RES/2
Y_CENTER = Y_RES/2
CREATE_TIME = str(time.time())
MAX_THROTTLE = 0.3 + THROTTLE_ZERO

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
def log_frames(out, image, j, accept_data):
    output_string = str(time.time()) + ", " + str(accept_data) + "\n"
    f = open("./logs/valid_frames-" + CREATE_TIME + ".txt", "a")
    f.write(output_string)
    f.close()
    # write output image to avi file
    out.write(image)
    # save a frame and backup video every 5 minutes (3240 frames)
    if j % 3240 == 0:
        print("saving frames...")
        cv2.imwrite("./frames/frame.bmp", image)
    # backup every minute (628 frames)
    if j % 648 == 0:
        print("backing up logs...")
        os.system("cp " + "./logs/valid_frames-" + CREATE_TIME + ".txt " + "./logs/backup-" + str(time.time()) + ".txt")
        print("backing up video...")
        os.system("cp " + "./videos/processed-" + CREATE_TIME + ".avi " + "./videos/backup-" + str(time.time()) + ".avi")      


## MAIN EXECUTION ##
if __name__ == '__main__':
    # change cwd to location of script
    os.chdir("/home/pi/FindTheSunFinal/") # TODO: maybe change starting directory?

    # set up servos
    kit = ServoKit(channels=16)
    # set the total range of servo in port 0 and 2 to 270
    kit.servo[0].actuation_range = 270
    kit.servo[2].actuation_range = 270
    # can tune the max and min pwm to max the servo go to desired angles
    kit.servo[0].set_pulse_width_range(500, 2500)
    kit.servo[2].set_pulse_width_range(500, 2500)
    kit.servo[0].angle = MAX_ANGLE/2
    kit.servo[2].angle = MAX_ANGLE/2
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
    out = cv2.VideoWriter("./videos/processed-" + CREATE_TIME + ".avi", cv2.VideoWriter_fourcc('M','J','P','G'), FPS, (X_RES,Y_RES), isColor=True)

    # set up signal handler
    signal.signal(signal.SIGINT, stop_servos)
    # set up loop counter
    j = 0
    # set up file
    f = open("./logs/valid_frames-" + CREATE_TIME + ".txt", "w+")
    f.write("header\n")
    f.close()

    start = time.perf_counter()
    # main loop
    while True:
        # get image and contours
        image, cnts = locate_sun()

        # if no countours, assume lost sun and start search protocol
        if cnts == []:
            print("LOST SUN :o")
            if throttle_last >= THROTTLE_ZERO:
                throttle_last = 0.2 + THROTTLE_ZERO
            else:
                throttle_last = -0.2 + THROTTLE_ZERO
            kit.continuous_servo[1].throttle = throttle_last
                
            while True:
                image, cnts = locate_sun()
                if cnts != []:
                    print("FOUND SUN")
                    throttle_last = THROTTLE_ZERO
                    kit.continuous_servo[1].throttle = throttle_last
                    break
                angle_lost = angle_last + (4 * SIGN)
                angle_last = angle_lost
                if angle_lost < 0:
                    angle_lost = 0
                    SIGN = -SIGN
                elif angle_lost > MAX_ANGLE:
                    angle_lost = MAX_ANGLE
                    SIGN = -SIGN

                print("angle_lost", angle_lost, "SIGN", SIGN, "THROTTLE", throttle_last)
                print("angle: " + str(angle_lost))
                kit.servo[0].angle = angle_lost
                kit.servo[2].angle = angle_lost
                
                if abs(throttle_last) < 0.08 + THROTTLE_ZERO:
                    throttle_last = 0.10 + THROTTLE_ZERO
                    kit.continuous_servo[1].throttle = throttle_last

                log_frames(out, image, j, 0)       
        
                j += 1
                time.sleep(0.2)
                
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
            SIGN = 1
        else:
            SIGN = -1
        
        # ensure angle is inbounds
        if angle_curr < 0:
            angle_curr = 0
        elif angle_curr > MAX_ANGLE:
            angle_curr = MAX_ANGLE

        # ensure throttle is inbounds
        if throttle_curr < -MAX_THROTTLE:
            throttle_curr = -MAX_THROTTLE
        elif throttle_curr > MAX_THROTTLE:
            throttle_curr = MAX_THROTTLE

        # on early iterations keep angle at midpoint
        # TODO: necessary??
        if j < 5:
            kit.servo[0].angle = MAX_ANGLE/2
            kit.servo[2].angle = MAX_ANGLE/2
        else:
            print("angle: " + str(angle_curr))
            kit.servo[0].angle = angle_curr
            kit.servo[2].angle = angle_curr + 5.0
            kit.continuous_servo[1].throttle = throttle_curr
            
        # update last angle + throttle
        angle_last = angle_curr
        throttle_last = throttle_curr
        
        print("P: ", Py, "I: ", Iy, "D: ", Dy, "throttle: ", throttle_curr)
        
        # calculate distance from sun and decide whether it's close enough
        accept_data = 1
        distance = math.sqrt(Errorx ** 2 + Errory ** 2)
        if distance >= MAX_RADIUS:
            accept_data = 0
        
        log_frames(out, image, j, accept_data)

        j += 1
        time.sleep(0.00)
