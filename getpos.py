from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import time

##################### CAMERA STUFF ##########################
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(1)

for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image = frame.array
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret, thresh_img = cv2.threshold(gray,100,255,0)
    contours, hierarchy = cv2.findContours(thresh_img,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    for i in range(0,len(contours)):
        c = contours[i]
        approx = cv2.approxPolyDP(c,0.1*cv2.arcLength(c,True),True)
        val = False
        if hierarchy[0,i,2] > -1 and cv2.contourArea(contours[i])>0 and cv2.contourArea(contours[i+1])>0:
            ratio = cv2.contourArea(contours[i])/cv2.contourArea(contours[i+1])
            if  ratio < 1.5 and ratio >1.25 :
                val = True
        if len(approx)==4 and val == True:
            M = cv2.moments(c)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #print("("+str(cx)+","+str(cy)+")")
            cv2.drawContours(image, [c], -1, (0,0,255), 2)
            x,y,w,h = cv2.boundingRect(c)
            cv2.putText(image,"("+str(cx)+","+str(cy)+")"+"-------"+str(cv2.contourArea(contours[i])), (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            cv2.imshow("FRAME",image)
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord("q"):
        break
