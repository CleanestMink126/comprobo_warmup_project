#For making Neato follow avoid obstacles
#Comp Robo 2018 Warm-Up Project

import numpy as np
import interface
import teleop
import rospy

def getangle90(angle):
    """Wraps angles around
    Returns an upper and lower value 45 degrees away from original input (90 degree FoV)"""
    wrapped1 = True
    wrapped2 = False
    angleLow = angle-45
    if angleLow <0:
        wrap1 = True
        angleLow+=360
    angleHigh = angle+45
    if angleHigh >360:
        wrap2 = True
        angleHigh = angleHigh-360
    return(angleLow,angleHigh,wrapped1,wrapped2)


def run(distance = 1.5, margin = .25):
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()
    base_s = .25 #base speed
    base_t = 1 # base turning rate
    prop = 100.0 #proportion to turn (higher = less turning)
    turned = 0 #flag for if turned to avoid already or not
    d2 = 100

    print(distance+margin)
    while not rospy.is_shutdown():
        rospy.sleep(.5) #limits sampling rate
        isObject, degree_index = mylidar.get_object()

        if isObject: #if valid data received, determine what to do next
            mytelC.myspeedctrl.send_speed(0,0)
            if degree_index <90 or degree_index >270: #object in front of Neato
                if degree_index < 90:
                    mytelC.turn_90degrees('left')
                    turned = 1
                    angleLow, angleHigh, wrap1, wrap2 = getangle90(90)
                elif degree_index > 270:
                    mytelC.turn_90degrees('right')
                    turned = 2
                    angleLow, angleHigh, wrap1, wrap2 = getangle90(270)
                mytelC.myspeedctrl.send_speed(base_s,0)
                print(angleLow, angleHigh)
                while isObject:
                    isObject, _ = mylidar.get_object(angle_range=[angleLow,angleHigh])
                mytelC.turn_90degrees(turned)
                turned = 0
            else: #clear path state
                mytelC.myspeedctrl.send_speed(base_s,0)

        else:
            mytelC.myspeedctrl.send_speed(base_s,0) #if no wall found, go straight
    mytelC.myspeedctrl.send_speed(0,0) #when shut down, stop Neato from moving



if __name__ == "__main__":
    run()
