#For making Neato follow along a wall
#Comp Robo 2018 Warm-Up Project


import numpy as np
import interface
import teleop
import rospy

def run():
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep() #limit sampling rate

        #returns (degree_index int, distance float) follows equation OR (None, None)
        degree_index, d = mylidar.get_wall()
        if degree_index != None: #if valid data received, determine what to do next
            if d >= 1.5: #far distance state
                if degree_index >= 180:
                    mytelC.myspeedctrl.send_speed(.25,-.1+((degree_index-360)/100.0))
                elif degree_index < 180:
                    mytelC.myspeedctrl.send_speed(.25,.1+(degree_index/100.0))
            elif d < 1.5: #close distance state
                if degree_index <= 180:
                    if degree_index <= 90:
                        mytelC.myspeedctrl.send_speed(.25,-.1+((degree_index-90)/100.0))
                    else:
                        mytelC.myspeedctrl.send_speed(.25,.1+((degree_index-90)/100.0))
                elif degree_index > 180:
                    if degree_index <= 270:
                        mytelC.myspeedctrl.send_speed(.25,-.1+((degree_index-270)/100.0))
                    else:
                        mytelC.myspeedctrl.send_speed(.25,.1+((degree_index-270)/100.0))
        else:
            mytelC.myspeedctrl.send_speed(.25,0) #if no wall found, go straight
    mytelC.myspeedctrl.send_speed(0,0) #when shut down, stop Neato from moving

if __name__ == "__main__":
    run()
