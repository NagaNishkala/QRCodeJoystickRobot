#!/usr/bin/env python3
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import matplotlib.pyplot as plt
import math

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_PLAIN

def img2imgpoints(image_name):
    #Load images
    frame=cv2.imread(image_name)
    frame1=cv2.imread(image_name)

    lower_blue = np.array([255,0,0])
    upper_blue = np.array([255,0,0])

    # Converts the given image into a masked (binary-black/white) image where pixels within range are converted to white and rest to black
    mask = cv2.inRange(frame, lower_blue, upper_blue)

    #Get contours or boundaries between two sets
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    #Get moments of the masked image, using the maxmimum contour, we use the below portion of the code
    #To get max contour index
    #k = contours.index(max(contours, key = len)) 'Doesnt work at all times'
    sorted_contours= sorted(contours, key=cv2.contourArea, reverse= True)
    
    if len(sorted_contours)==0:
        return 'No QR Code found' 
    
    largest_item= sorted_contours[0]
    cv2.drawContours(frame, largest_item, -1, (0,255,0),10)

    #Store max contour 
    cnt = sorted_contours[0]

    #Obtain moment M and hence centroid
    M = cv2.moments(cnt)
    cx = int(M['m10']/M['m00'])
    cy = int(M['m01']/M['m00'])

    #Visualize center
    cv2.circle(frame, (cx, cy), 3, [0, 0, 255], 2)
    #cv2.imshow('contcen',frame)
    a=image_name.split('.')
    str1=a[0]+'_contour_cen.png'
    cv2.imwrite(str1,frame)
    
    cv2.line(frame,(0,240),(640,240),(255,255,0))
    cv2.line(frame,(320,0),(320,480),(255,255,0))
    
    dist=((cx-320)**2 + (cy-240)**2)**(0.5)
    dist_str="The distance to point is "+str(dist)
    cv2.putText(frame, dist_str, (50, 50), font, 2,(255, 0, 0), 1)

    x1,y1=(cx-320),(cy-240)
    ang=math.atan2(y1,x1) * (180/math.pi)

    angle_str="The angle to point is "+str(ang)
    cv2.putText(frame, angle_str, (50, 100), font, 2,(255, 0, 0), 1)
    
    cv2.line(frame,(cx,cy),(320,240),(255,0,255))
    
    cv2.imshow('QRCodeDetection2',frame)

    center_blue=[cx,cy]
    return center_blue

while True:
    _, frame = cap.read()
    frame=cv2.flip(frame, 1)
    decodedObjects = pyzbar.decode(frame)

    for obj in decodedObjects:
        pts = np.array([obj.polygon],np.int32)
        pts = pts.reshape((-1,1,2))
        cv2.polylines(frame,[pts],True,(255,0,0),4)
        #print('Points:',pts)

    cv2.imwrite('QRCodeDetection.png',frame)
    i2ip=img2imgpoints('QRCodeDetection.png')
    cv2.putText(frame, str(i2ip), (50, 50), font, 2,(255, 0, 0), 3)
    cv2.imshow('QRCodeDetection1',frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
