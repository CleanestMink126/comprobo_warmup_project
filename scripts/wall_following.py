#For making Neato follow along a wall
#Comp Robo 2018 Warm-Up Project


import numpy as np
import interface
import teleop
import rospy
from math import pi
import math

degrees2rad = pi/180

def run(distance = .75, margin = .25):
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()
    mymarker = interface.SendLineMarker()
    mybump = interface.ReceiveBump()
    marker_width = 1
    base_s = .15 #base speed
    base_t = .1 # base turning rate
    prop = 100.0 #proportion to turn (higher = less turning)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep() #limit sampling rate
        #returns (degree_index int, distance float) follows equation OR (None, None)
        if mybump.get_bump():#return if hit
            break
        degree_index, d = mylidar.get_wall()
        if degree_index != None: #if valid data received, determine what to do next
            x, y = d * math.cos(degrees2rad * degree_index),  d * math.sin(degrees2rad * degree_index)
            x_b = marker_width * math.cos(degrees2rad * degree_index + math.pi/2) + x
            y_b = marker_width * math.sin(degrees2rad * degree_index  + math.pi/2) + y
            x_e = marker_width * math.cos(degrees2rad * degree_index - math.pi/2) + x
            y_e = marker_width * math.sin(degrees2rad * degree_index  - math.pi/2) + y
            mymarker.update_marker([(x_b,y_b),(x,y),(x_e,y_e)])
            print(degree_index,d)
            '''The next part is just some logic we drew out about which
            ways to turn depending on distance and which direction the wall is'''
            if d > distance + margin: #far distance state
                if 180 <= degree_index < 360 :
                    mytelC.myspeedctrl.send_speed(base_s,-base_t+((degree_index-360)/100.0))
                elif 0 < degree_index < 180:
                    mytelC.myspeedctrl.send_speed(base_s,base_t+(degree_index/100.0))
            elif d < distance - margin: #close distance state
                if 180 <= degree_index < 360 :
                    mytelC.myspeedctrl.send_speed(base_s,base_t+((degree_index-180)/100.0))
                elif 0 < degree_index < 180:
                    mytelC.myspeedctrl.send_speed(base_s,-base_t+((degree_index-180)/100.0))
            else: #goood distance
                if degree_index < 180:
                    if degree_index <= 90:
                        mytelC.myspeedctrl.send_speed(base_s,-base_t+((degree_index-90)/100.0))
                    else:
                        mytelC.myspeedctrl.send_speed(base_s,base_t+((degree_index-90)/100.0))
                elif degree_index > 180:
                    if degree_index <= 270:
                        mytelC.myspeedctrl.send_speed(base_s,-base_t+((degree_index-270)/100.0))
                    else:
                        mytelC.myspeedctrl.send_speed(base_s,base_t+((degree_index-270)/100.0))
        else:
            mytelC.myspeedctrl.send_speed(.1,0) #if no wall found, go straight
    mytelC.myspeedctrl.send_speed(0,0) #when shut down, stop Neato from moving
    return degree_index

if __name__ == "__main__":
    run()
