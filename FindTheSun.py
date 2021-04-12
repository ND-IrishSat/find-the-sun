import os
os.chdir('/home/pi/Documents/PythonCodes/')
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
from time import sleep
import math
import signal

# set up signal handler for ctrl+c
def stop_servos(signum, sf):
    kit.continuous_servo[1].throttle = 0
    print("stop")
    exit()

signal.signal(signal.SIGINT, stop_servos)
#Setup the Servos (x servo (azimuth) into pin 11) (y servo into ___)
#GPIO.setmode(GPIO.BOARD)
#xpin = 12
#ypin = 11
#set pin 11 as output, and set servo1 as pin 11 as PWM
#GPIO.setup(xpin,GPIO.OUT)
#xservo = GPIO.PWM(xpin,50) #11 is pin, 50 is 50Hz pulse
#GPIO.setup(ypin,GPIO.OUT)
#yservo = GPIO.PWM(ypin,50)
#xservo.start(0)
#yservo.start(0)
maxangle = 80
kit = ServoKit(channels=16)
kit.servo[0].actuation_range = maxangle #set the total range of servo in port 0 to 60
kit.servo[0].set_pulse_width_range(1000, 2000) #can tune the max and min pwm to max the servo go to desired angles
kit.servo[2].actuation_range = maxangle #set the total range of servo in port 0 to 60
kit.servo[2].set_pulse_width_range(1000, 2000) #can tune the
kit.servo[0].angle = maxangle/2
time.sleep(0.3)
kit.continuous_servo[1].throttle = 1
time.sleep(0.01)
kit.continuous_servo[1].throttle = 0

GPIO.setwarnings(False) # Ignore warning for now
#GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(14, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
# Set pin 10 to be an input pin and set initial
#value to be pulled low (off)

Ix = 0
Iy = 0
Errorxlast = 0
Errorylast = 0
anglelast = maxangle/2
throttlelast = 0
xRes = 640
yRes = 480
fps = 30
THRESHHOLD = 150
SIGN = 1
l = 0
throttle1 = 0
#camera = PiCamera()
#camera.rotation = 180  #sets the rotation of the image #will be 180
#camera.resolution = (xRes, yRes)
vs = VideoStream(usePiCamera=False, resolution=(xRes,yRes)).start()
#vs = imutils.rotate(vsUnrotated, 180, scale=1)
time.sleep(0.3)
# TODO redo naming so that it won't overwrite old file if crashes
out = cv2.VideoWriter('processed.avi', cv2.VideoWriter_fourcc('M','J','P','G'), fps, (xRes,yRes),isColor=True)
j = 0

while True:
    m = 0
    if GPIO.input(15) == GPIO.HIGH or GPIO.input(14) == GPIO.HIGH:
        print("Button was pushed!")
        if abs(throttlelast) >= 0.15:
            kit.continuous_servo[1].throttle = -throttlelast
            throttlelast = -throttlelast
        else:
            if throttlelast < 0:
                kit.continuous_servo[1].throttle = 0.15
                throttlelast = 0.15
            elif throttlelast >= 0:
                kit.continuous_servo[1].throttle = -0.15
                throttlelast = -0.15
        time.sleep(2)
    BlurRadius =11
    #camera.start_preview(alpha=255)
    #sleep(0.1)
    #camera.capture('cameracapture.jpg') #this writes over the old file
    image = vs.read()
    #camera.stop_preview()
    #image = cv2.imread('cameracapture.jpg')
    #orig = image.copy()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("Gray", gray)
    blurred = cv2.GaussianBlur(gray, (BlurRadius,BlurRadius), 0)
    #cv2.imshow("Blurred", blurred)
    #Threshold the Image
    minthres = 242
    setpixelsto = 255 #white
    thresh = cv2.threshold(blurred, THRESHHOLD, setpixelsto, cv2.THRESH_BINARY)[1]
    #cv2.imshow("Thresh", thresh)
    thresh = cv2.erode(thresh, None, iterations=2)
    thresh = cv2.dilate(thresh, None, iterations=4)

    labels = measure.label(thresh, background=0, connectivity=2)
    mask = np.zeros(thresh.shape, dtype="uint8")
    numPixelMax=0

    for label in np.unique(labels):
        if label == 0:
            continue
        labelMask = np.zeros(thresh.shape, dtype="uint8")
        labelMask[labels == label] = 255
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
        labelMask[labels == label] = 255

        # if the number of pixels in the component is sufficiently
        # large, then add it to our mask of "large blobs"
        if label == placement:
            mask = cv2.add(mask, labelMask)
    #cv2.imshow("New", mask)
    #mask = cv2.add(mask, labelMask[labels == placement] = 255)

    # find the contours in the mask, then sort them from left to
    # right
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    # TODO: check if empty
    if cnts == []:
        print("LOST SUN :o")
        if throttlelast >= 0:
            throttlelast = 0.1
            kit.continuous_servo[1].throttle = 0.1
        else:
            throttlelast = -0.1
            kit.continuous_servo[1].throttle = -0.1
            
        while True:
            image = vs.read()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (BlurRadius,BlurRadius), 0)
            setpixelsto = 255 #white
            thresh = cv2.threshold(blurred, THRESHHOLD, setpixelsto, cv2.THRESH_BINARY)[1]
            thresh = cv2.erode(thresh, None, iterations=2)
            thresh = cv2.dilate(thresh, None, iterations=4)

            labels = measure.label(thresh, background=0, connectivity=2)
            mask = np.zeros(thresh.shape, dtype="uint8")
            numPixelMax=0

            for label in np.unique(labels):
                if label == 0:
                    continue
                labelMask = np.zeros(thresh.shape, dtype="uint8")
                labelMask[labels == label] = 255
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
                labelMask[labels == label] = 255

                # if the number of pixels in the component is sufficiently
                # large, then add it to our mask of "large blobs"
                if label == placement:
                    mask = cv2.add(mask, labelMask)
            #cv2.imshow("New", mask)
            #mask = cv2.add(mask, labelMask[labels == placement] = 255)

            # find the contours in the mask, then sort them from left to
            # right
            cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
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
            elif anglelost >80:
                anglelost = 80
                SIGN = -SIGN
            else:
                anglelost = anglelost
            print("anglelost", anglelost, "SIGN", SIGN, "THROTTLE", throttlelast)
            #out.write(thresh)
            out.write(image)
            #kit.servo[0].angle = 4*(math.cos(angle)+1)
            #angle += math.pi/4
            kit.servo[0].angle = anglelost
            
            if abs(throttlelast) < 0.08:
                throttlelast = 0.10
                kit.continuous_servo[1].throttle = throttlelast
            
            if GPIO.input(15) == GPIO.HIGH or GPIO.input(14) == GPIO.HIGH:
                print("Button was pushed!")
                if abs(throttlelast) >= 0.15:
                    kit.continuous_servo[1].throttle = -throttlelast
                    throttlelast = -throttlelast
                else:
                    if throttlelast < 0:
                        kit.continuous_servo[1].throttle = 0.15
                        throttlelast = 0.15
                    elif throttlelast >= 0:
                        kit.continuous_servo[1].throttle = -0.15
                        throttlelast = -0.15
                time.sleep(2)
            time.sleep(0.2)
            
    cnts = contours.sort_contours(cnts)[0]

    # loop over the contours
    for (i, c) in enumerate(cnts):
        # draw the bright spot on the image
        (x, y, w, h) = cv2.boundingRect(c)
        ((cX, cY), radius) = cv2.minEnclosingCircle(c)
        #print(cX, cY)
        cv2.circle(image, (int(cX), int(cY)), int(radius),
            (0, 0, 255), 3)
        #cv2.putText(image, "#{}".format(i + 1), (x, y - 15),cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
    
    # show the output image
    #cv2.imshow("Image", image)
    #cv2.waitKey(0)
    #cv2.imwrite('thresh' + str(j) + '.png', thresh)
    out.write(image)

    xcenter = xRes/2
    ycenter = yRes/2
    #cv2.circle(image, (int(xcenter), int(ycenter)), 5, (0, 0, 255), 3)

    # Lets do some PID shiz
    Errorx = cX-xcenter
    Errory = cY-ycenter

    Pvalx = -0.0006
    Pvaly = 0.01
    Px = Pvalx * Errorx
    Py = Pvaly * Errory   
    Ivalx = -0.00009
    Ivaly = 0.0001
    Ix = Ix + ((Errorx)*Ivalx)
    Iy = Iy + ((Errory)*Ivaly)
    Dvalx = 0.000
    Dvaly = -0.003
    Dx = (Errorxlast - Errorx)*Dvalx
    Dy = (Errorylast - Errory)*Dvaly
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
    elif angle0 >80:
        angle0 = 80
    else:
        angle0 = angle0
    if throttle1 < -0.2:
        throttle1 = -0.2
    elif throttle1 >0.2:
        throttle1 = 0.2
    else:
        throttle1 =throttle1
    if j<5:
        kit.servo[0].angle = maxangle/2

    else:
        kit.servo[0].angle = angle0
        kit.continuous_servo[1].throttle = throttle1     
    anglelast = angle0
    throttlelast = throttle1
    #kit.servo[0].angle = angle0
    
    print("P: ", Px, "I: ", Ix, "D: ", Dx, "throttle: ", throttle1)
    #kit.servo[0].angle = angle

    #print("sun coordinates: ", cX,",", cY)
    #print(Px, Py)

    

    #if Px < -5:
     #   xDutytmp = -1
    #elif Px > 5:
     #   xDutytmp = 1
    #else:
    #    xDutytmp = Px
    #if Py < -5:
    #    yDutytmp = -1
    #elif Py > 5:
    #    yDutytmp = 1
    #else:
    #    yDutytmp = Py
    
    #xDuty = xDuty + xDutytmp
    #yDuty = yDuty + yDutytmp
    #if xDuty > 10:
    #    xservo.ChangeDutyCycle(10)
    #elif xDuty < 4:
     #   xservo.ChangeDutyCycle(4)
    #else:
    #    xservo.ChangeDutyCycle(xDuty)
    #if yDuty > 10:
    #    yservo.ChangeDutyCycle(10)
    #elif yDuty < 4:
    #    yservo.ChangeDutyCycle(4)
    #else:
    #    yservo.ChangeDutyCycle(xDuty)
    #print(xDutytmp, yDutytmp)
    #print(xDuty, yDuty)
    j = j+1
    
 
    time.sleep(0.00)

#xservo.stop()
#yservo.stop()
#GPIO.cleanup()
kit.continuous_servo[1].throttle = 0   




