#!/usr/bin/env python

import numpy as np
import cv2
import sys

ORIG_WIDTH = 160
ORIG_HEIGHT = 120
RESIZE_WIDTH = 640
RESIZE_HEIGHT = 480

captures = []

#for i in range(1, len(sys.argv)):
#    captures.append(cv2.VideoCapture(int(sys.argv[i])))

for i in range(1, len(sys.argv)):
    temp = cv2.VideoCapture(int(sys.argv[i]))
    temp.set(cv2.CAP_PROP_FRAME_WIDTH, ORIG_WIDTH)
    temp.set(cv2.CAP_PROP_FRAME_HEIGHT, ORIG_HEIGHT)
    captures.append(temp)


#cap1 = cv2.VideoCapture(int(sys.argv[1]))
#cap2 = cv2.VideoCapture(int(sys.argv[2]))

while(True):
 
    frames = []

    for i in range(len(captures)):
        _, frame = captures[i].read()
	frames.append(cv2.resize(frame, (RESIZE_WIDTH,RESIZE_HEIGHT)))
    
    # Capture frame-by-frame
#    _, frame1 = cap1.read()
#    _, frame2 = cap2.read()

    for i in range(len(frames)):
        cv2.imshow('cam ' + str(i) + ' ', frames[i])


    # Display the resulting frame
#    cv2.imshow('cam1',frame1)
#    cv2.imshow('cam2',frame2)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

for capture in captures:
    capture.release()

# When everything done, release the capture
#cap1.release()
#cap2.release()
cv2.destroyAllWindows()
