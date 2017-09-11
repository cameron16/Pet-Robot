"""
    Filename: rolling_control.py
    Created by Judy Stephen (jls633), Cameron Boroumand (cb596)
    Created on: November 28, 2016
    Last Updated: December 10, 2016
"""

"""
Note: Comments in this code use "ball" and "object" interchangeably. 
They are equivalent in meaning 
"""
import os
import RPi.GPIO as GPIO
import time
import sys
#import pygame
#from pygame.locals import *

# import the necessary picamera and opencv packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np

#import stuff for voice recognition
import pyttsx
import subprocess                 # useful to run espeak on terminal
import datetime                   # useful to get today's date
import speech_recognition as sr   # speech recognition engine
import pyaudio                    # needed to record microphone 
import wave                       # needed to save as wav file

from os import path
AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "user.wav")

GPIO.setmode(GPIO.BCM) #set for broadcom numbering

#initalize the camera 
camera = PiCamera()
camera.resolution = (640,480)
camera.framerate = 32

#set up state variable
listeningState = False

#def left servo pin
leftServo=6

#def right servo pin
rightServo = 5

#set up left servo pin
GPIO.setup(leftServo,GPIO.OUT) #set up left pin as output
leftChannel = leftServo
leftFrequency =50 #hz
leftP = GPIO.PWM(leftChannel,leftFrequency)

#set up right servo pin
GPIO.setup(rightServo, GPIO.OUT) #set up right pin as output
rightChannel = rightServo
rightFrequency = 50 #hz
rightP = GPIO.PWM(rightChannel,rightFrequency)

maxSpeedClockwise =6.63 
maxSpeedCounterClockwise = 7.8

maxSpeedRightForward = maxSpeedClockwise
maxSpeedRightBackward = maxSpeedCounterClockwise
maxSpeedLeftForward = maxSpeedCounterClockwise
maxSpeedLeftBackward = maxSpeedClockwise

#stop the servo
def stopServo(servo):
    dc = 0#7.18
    if servo == rightServo:
        rightP.start(dc)
        #print "right servo stopped"
    if servo == leftServo:
        leftP.start(dc)
        #print "left servo stopped"

#move servo forward at max speed
def moveForward(servo):
    if servo == rightServo:
        dc = maxSpeedRightForward
        rightP.start(dc)
        #print "right servo forward"
    if servo == leftServo:
        dc = maxSpeedLeftForward
        leftP.start(dc)

#move servo backward at max speed
def moveBackward(servo):
    if servo == rightServo:
        dc = maxSpeedRightBackward
        rightP.start(dc)
    if servo == leftServo:
        dc = maxSpeedLeftBackward
        leftP.start(dc)

def moveForwardVoice():
    startingTime = time.clock()
    while (time.clock() - startingTime < 5):
        moveRobotForward()
        if GPIO.input(longDist): #if there is object in front of you
            time.sleep(1) #move forward a bit more
            stopRobot()
            break
    stopRobot()

def moveBackwardVoice():
    startingTime = time.clock()
    while (time.clock() - startingTime < 5):
        moveRobotBackward()
        if GPIO.input(longDist): #if there is object in front of you
            time.sleep(1) #move backward a bit more
            stopRobot()
            break
    stopRobot()

def turnRightVoice():
    pivot90Right()
    moveForwardVoice()

def turnLeftVoice():
    pivot90Left()
    moveForwardVoice()

def moveRobotForward():
    moveForward(rightServo)
    moveForward(leftServo)

def moveRobotBackward():
    moveBackward(rightServo)
    moveBackward(leftServo) 

def stopRobot():
    stopServo(leftServo)
    stopServo(rightServo)

def pivot90Right():
    moveForward(leftServo)
    moveBackward(rightServo)
    time.sleep(0.6)
    stopRobot()
    
def pivot90Left():
    moveBackward(leftServo)
    moveForward(rightServo)
    time.sleep(0.6)
    stopRobot()

def pivot45Left():
    moveBackward(leftServo)
    moveForward(rightServo)
    time.sleep(1)
    stopRobot()

def pivot45Right():
    moveForward(leftServo)
    moveBackward(rightServo)
    time.sleep(1)
    stopRobot()

def pivot30Right():
    moveForward(leftServo)
   # moveBackward(rightServo)
    stopServo(rightServo)
    time.sleep(0.65) #was 0.5
    stopRobot()

def veerLeft():
    #max speed on right wheel
    #slow on left wheel
    moveForward(rightServo)
    stopServo(leftServo)
    time.sleep(0.2)
    stopRobot()

def veerRight():
    #max speed on left wheel
    #slow on right wheel
    moveForward(leftServo)
    stopServo(rightServo)
    time.sleep(0.2)

    stopRobot()
        
#set up exit button
pinNum = 17
GPIO.setup(pinNum,GPIO.IN,pull_up_down=GPIO.PUD_UP)

#set up interrupt on exit button 
def GPIO17_callback(channel):
    print "falling edge detected on port 17"
    exitProgram()
GPIO.add_event_detect(17,GPIO.FALLING,callback=GPIO17_callback,bouncetime=300)

#on exiting program (when exit button itnterrupt is called)
def exitProgram():
    print "exiting program"
    stopRobot()
    GPIO.cleanup()
    sys.exit()

#set up start button
startButton = 22
GPIO.setup(startButton, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#set up interrupt on start button 
def GPIO22_callback(channel):
    print "Listening now"
    global listeningState
    stopRobot()
    listeningState = True
GPIO.add_event_detect(22,GPIO.FALLING,callback=GPIO22_callback,bouncetime=300)


#set up long distance sensor
longDist = 19
GPIO.setup(longDist, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


#set up LED on pin 13
ledPin = 13
GPIO.setup(ledPin, GPIO.OUT)
ledFreq = 10
ledP = GPIO.PWM(ledPin,ledFreq)
ledDC = 50.0

#define simple on and off LED functions
def turnLedOn():
    ledP.start(50)

def turnLedOff():
    ledP.start(0)

#this function is called once robot has found object/ball
def atBall(color):
    stopRobot()
    turnLedOn()
    time.sleep(5) #flash LED for 5 seconds
    turnLedOff()
    if color == "blue": 
        fetchVoice("green") #now go find green object

"""
This function is called when the robot is in motion
towards the ball
Returns x, 900, 1000
x: returns x_coordinate position of object
900: ball is extremely to robot
1000: ball is not in vision of robot
"""
def findBall(color, objectStatus):
    #look for ball
    #grab a reference to the raw camera capture
    rawCapture = PiRGBArray(camera,size=(640,480))
    #allow the camera to warmup
    time.sleep(0.1)
    #capture frames from the camera
    for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
        image=frame.array
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        lower_green = np.array([29,86,6])
        upper_green = np.array([64,255,255])

        # Threshold the HSV image to get either only blue or only green colors
        if color == "blue":
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        else:
            mask = cv2.inRange(hsv, lower_green, upper_green)
        
        mask = cv2.erode(mask,None,iterations=2)
        mask=cv2.dilate(mask,None,iterations=2)

        cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        if len(cnts) > 0:
            c = max(cnts,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if objectStatus == "notVeryClose": 
                if color == "blue":
                    threshold = 20 #larger threshold for blue b/c using larger blue object
                else:
                    threshold = 10 #smaller threshold for green b/c using smaller green object
                if radius > 175:
                    return 900 #signal that robot is very close to object
                elif radius >threshold: 
                    return x
                else:
                    return 1000 #radius too small to be valid
            else:
                if radius > 100:#10:
                    return x #ball is infront of us
                else:
                #doesn't make sense for code to fall in this
                #b/c radius should not be <100 at this point,
                #so just stop the robot since something is wrong
                    return 1000 #signal to stop robot
        else:
            print 1000 
            return 1000 #object is not in camera's view
                   
        #cv2.imshow('frame',image)

        #cv2.imshow('mask',mask)

        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)

        if key == ord("q"):
            #camera.release()
            cv2.destroyAllWindows()
            exitProgram()

        
"""
Target is already found. keep going towards it
until either the distance sensor sees it
OR until camera loses it, whichever occurs first
"""
def moveUntilLost(color):
    xPos = findBall(color,"veryClose")
    if xPos == 1000: #ball lost, just stop
        atBall(color)
    elif GPIO.input(longDist):
        print "we are at object"
        atBall(color)
    elif (xPos<480 and xPos > 160): #ball is straight ahead
        print "found ball, go straight"
        moveRobotForward()
        moveUntilLost(color)
    elif (xPos < 160): #ball is to the left
        print "found ball, veer left"
        veerLeft()
        moveUntilLost(color)
    else: #ball is to the right
        print "found ball, veer right"
        veerRight()
        moveUntilLost(color)
     
"""
This function is called when robot is to be moving towards
the object and constantly checking the object's location relative
to itself so the robot can continously adjust its motion
"""
def moveToTarget(color):
    xPos = findBall(color, "notVeryClose") 
    if xPos == 1000: #ball is not in camera's sight, just stop and go back to polling
        stopRobot()
        fetchVoice(color) #look for object again
    elif xPos ==900: #ball is right infront of robot, go for a bit more until we lose the object
        print "code 900"
        moveUntilLost(color)
    elif GPIO.input(longDist): #distance sensor senses the object. go for a bit more until we lose the object
        print "at object"
        moveUntilLost(color)
    elif (xPos < 480 and xPos > 160): #ball is straight ahead
        print "go straight"
        moveRobotForward()
        moveToTarget(color)
    elif (xPos < 160): #ball is to the left
        print "veer left"
        veerLeft()
        moveToTarget(color)
    else: #ball is to the right
        print "veer right" 
        veerRight()
        moveToTarget(color)

"""
This function is called when robot is sotpped and looking for
the object
returns 0,1, or 2:
0: no ball found
1: ball found, robot is not too close to ball
2: ball found. robot is extremely close to ball
"""
def pollBall(startTime, color):
    #look for ball
    #runs until 1.3 seconds elapses or ball is found, whichever comes first
    #grab a reference to the raw camera capture
    rawCapture = PiRGBArray(camera,size=(640,480))
    #allow the camera to warmup
    time.sleep(0.1)
    global ballFound
    global ballXPos
    global ballYPos
    global pollingState
    #capture frames from the camera
    for frame in camera.capture_continuous(rawCapture,format="bgr",use_video_port=True):
        currentTime = time.clock()
        if currentTime - startTime > 1.3:
            return 0 #no ball found
        image=frame.array
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # define range of blue color in HSV
        lower_blue = np.array([110,50,50])
        upper_blue = np.array([130,255,255])

        #define rnage of green color in HSV
        lower_green = np.array([29,86,6])
        upper_green = np.array([64,255,255])

        # Threshold the HSV image to get either only blue or only green 
        if color == "blue":
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
        else:
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
        mask = cv2.erode(mask,None,iterations=2)
        mask=cv2.dilate(mask,None,iterations=2)

        cnts = cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(cnts) > 0:
            c = max(cnts,key=cv2.contourArea)
            ((x,y),radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            if color == "blue":
                threshold = 20 #larger threshold for blue b/c using larger blue object
            else:
                threshold = 10 #smaller threshold for green b/c using smaller green object
            if radius > threshold:
                if radius > 175: #object/ball is right next to us!
                    print "found it next to us"
                    return 2 #ball saturation
                else: 
                    print "found a ball - leave poll"
                    return 1 #ball is found but not right next to robot              
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)

        if key == ord("q"):
            cv2.destroyAllWindows()
            exitProgram()
        
def fetchVoice(color):
    if color == "blue":
        print "looking for some blue now"
    else:
        print "looking for some green now"
    startTime = time.clock()
    myFoundBall = pollBall(startTime, color)
    if myFoundBall == 1: #ball has been located, so move towards it
        moveToTarget(color)
    elif myFoundBall == 0: #no ball was found
        print "pivoting now"
        pivot30Right() #pivot ~30 degrees 
        fetchVoice(color) #look for object again
    else:
        print "found ball while polling" #ball/object is right next to robot
        moveUntilLost(color)
        
"""Say today's date """
def sayDate():
    myDateList = []
    today = datetime.date.today()
    myDateList.append(today)
    formatedDate =  today.strftime("It is %A, %B %d %Y") # 'It is Saturday, December 03 2016'
    print formatedDate
    # speak the date 
    speakStr(formatedDate)

""" Records user input from microphone for 3 seconds and saves in wave file user.wav"""
def record_audio():
    CHUNK = 8192
    FORMAT = pyaudio.paInt16
    CHANNELS = 1  # used to be 2 but our microphone only has 1
    RATE = 44100
    RECORD_SECONDS = 3
    WAVE_OUTPUT_FILENAME = "user.wav"

    p = pyaudio.PyAudio()

    stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK)

    print "* Recoding audio..."

    frames = []
    
    for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
        data = stream.read(CHUNK)
        frames.append(data)

    print "* done\n"

    stream.stop_stream()
    stream.close()
    p.terminate()

    wf = wave.open(WAVE_OUTPUT_FILENAME, "wb")
    wf.setnchannels(CHANNELS)
    wf.setsampwidth(p.get_sample_size(FORMAT))
    wf.setframerate(RATE)
    wf.writeframes(b''.join(frames))
    wf.close()

""" Recognize user speech by recognizing the speech from wav file user.wav
    Currently uses Google Speech Recognition because it is much faster than Sphinx 
    Return: recognized speech or "error_recognizing" if there was an error """
def recognize_speech():
    recognized_speech = "initialized recognized_speech"
    r = sr.Recognizer()
    with sr.AudioFile(AUDIO_FILE) as source:
        audio = r.record(source)      # read the entire audio file

    start = time.clock()
    try:
        recognized_speech = r.recognize_google(audio)
        print("Google Speech Recognition thinks you said " + recognized_speech)
    except sr.UnknownValueError:
        print("Google Speech Recognition could not understand your input")
    except sr.RequestError as e:
        print("Could not request results from Google Speech Recognition serivce; {0}".format(e))
    end = time.clock()
    timeElapsed = end - start
    print "computation time for Google Speech Recognition : " + str(timeElapsed)
    return recognized_speech

song_wf = 0
WAVE_FILE_NAME = path.join(path.dirname(path.realpath(__file__)), "discovery.wav")

def play_song_callback(in_date, frame_count, time_info, status):
    data = song_wf.readframes(frame_count)
    return (data, pyaudio.paContinue)

""" Play a wave file """
def play_song():
    # WAVE_FILE_NAME = "discovery.wav"
    song_wf = wave.open(WAVE_FILE_NAME, 'rb')

    p = pyaudio.PyAudio()
    stream = p.open(format=p.get_format_from_width(song_wf.getsampwidth()),
                    channels=song_wf.getnchannels(),
                    rate=song_wf.getframerate(),
                    output=True,
                    stream_callback=play_song_callback)
    stream.start_stream()
    while stream.is_active():
        time.sleep(0.1)
    stream.stop_stream()
    stream.close()
    song_wf.close()
    print "play song stream closed"
    p.terminate()

"""Input: the recognized user command
   This function will find what the user wants to do and perform it
   For example, if the user says "move forward" then it will move the 
   robot forward. 
   Supported commands: move forward, move back, left, right, stop, play song"""
def command_robot(recognized_command):
    global pollingState
    global movingState
    global doneState
    if "forward" in recognized_command:
        # move forward
        print "move robot forward"
        moveForwardVoice()
    elif "move back" in recognized_command:
        # move backward
        print "move robot backward"
        moveBackwardVoice()
    elif "left" in recognized_command:
        # move left
        turnLeftVoice()
        print "move robot left"
    elif "right" in recognized_command:
        # move right
        print "move robot right"
        turnRightVoice()
    elif "stop" in recognized_command:
        # stop robot
        print "stop the robot"
    elif "song" in recognized_command:
        print "play a song"
        #play_song()
    elif "ball" in recognized_command:
        # fetch command 
        print "robot in fetch mode"
        fetchVoice("blue")
    else: 
        print "unsupported command"

if __name__=="__main__":
    engine = pyttsx.init()
    try:
        while (True):
            if listeningState:
                listeningState = False
                print 'running pi_voice.py ...'
                record_audio()
                recognized_word = recognize_speech()
                command_robot(recognized_word)
                
    except KeyboardInterrupt:
        print "keyboard interrupt"

    exitProgram()  