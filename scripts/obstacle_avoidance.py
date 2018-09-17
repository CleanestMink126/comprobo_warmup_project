#For making Neato follow avoid obstacles
#Comp Robo 2018 Warm-Up Project

import numpy as np
import interface
import teleop
import rospy


def run():
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()
    base_s = .25 #base speed
    base_t = .1 # base turning rate
    prop = 100.0 #proportion to turn (higher = less turning)

    while not rospy.is_shutdown():
        r.sleep() #limits sampling rate
        r = rospy.Rate(10)
        degree_index, d = mylidar.get_wall(required_points=10)

        if degree_index != None: #if valid data received, determine what to do next
            print(degree_index,d)
            if d < distance + margin: #close distance state
                if degree_index < 90:
                    mytelC.myspeedctrl.send_speed(base_s,-base_t+((degree_index-90)/100.0))
                elif degree_index > 270:
                    mytelC.myspeedctrl.send_speed(base_s,base_t+((degree_index-270)/100.0))
            else: #goood distance
                mytelC.myspeedctrl.send_speed(base_s,base_s)

        else:
            mytelC.myspeedctrl.send_speed(.1,0) #if no wall found, go straight
    mytelC.myspeedctrl.send_speed(0,0) #when shut down, stop Neato from moving
