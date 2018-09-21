from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import time
import sys
import paho.mqtt.client as paho
broker = "192.168.43.254"
init = 0
#################### GET ARGUMENTS ##########################

xt = int(sys.argv[1])
yt = int(sys.argv[2])
targetpos = [xt,yt]

#################### MQTT CALLBACK ##########################

def on_message(client, userdata, message):
    global initpos
    rcv=message.payload.decode("utf-8")
    finalpos = [cx,cy]
    done,reward = compute(initpos,finalpos)
    initpos = finalpos
    if rcv == "rr":
        client.publish("outTopic",str(done)+" "+str(reward))

#################### COMPUTE REWARD #########################

def compute(initpos,finalpos):
    rewx = finalpos[0]-initpos[0]
    rewy = np.abs(finalpos[1]-initpos[1])
    if np.abs(rewx) < 2:
        rewx = 0
    if rewy < 2:
        rewy = 0
    if targetpos[0]-finalpos[0] <5:
        done = 1
    elif  targetpos[0]-finalpos[0] >5:
        done = 0
    return done, rewx-rewy


###################### MQTT STUFF ###########################

client= paho.Client("quadpod101")
client.on_message=on_message
print("connecting to broker ",broker)
client.connect(broker)
client.loop_start()
print("subscribing ")
client.subscribe("ack")

##################### CAMERA STUFF ##########################

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 20
rawCapture = PiRGBArray(camera, size=(640, 480))
time.sleep(1)

######################### CAPTURE FROM CAMERA ##############################

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
            a = cv2.contourArea(contours[i])
            ratio = a/cv2.contourArea(contours[i+1])
            if  ratio < 1.5 and ratio >1.25 :
                val = True
        if len(approx)==4 and val == True and a >5000:
            M = cv2.moments(c)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            if init == 0:
                startpos = [cx,cy]
                initpos = startpos
                print("Initial Position is: ",initpos)
                print("Target Position is: ",targetpos)
                init = 1
            #print("("+str(cx)+","+str(cy)+")")
            #cv2.drawContours(image, [c], -1, (0,0,255), 2)
            #x,y,w,h = cv2.boundingRect(c)
            #cv2.putText(image,"("+str(cx)+","+str(cy)+")", (x,y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
    key = cv2.waitKey(1) & 0xFF
    rawCapture.truncate(0)
    if key == ord("q"):
        break
