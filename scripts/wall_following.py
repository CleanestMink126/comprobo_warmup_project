#For making Neato follow along a wall
#Comp Robo 2018 Warm-Up Project


import numpy as np
import interface
import teleop
import rospy

def run(distance = 1.5, margin = .1):
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep() #limit sampling rate
        base_s = .25 #base speed
        base_t = .1 # base turning rate
        prop = 100.0 #proportion to turn (higher = less turning)
        #returns (degree_index int, distance float) follows equation OR (None, None)
        degree_index, d = mylidar.get_wall()
        if degree_index != None: #if valid data received, determine what to do next
            print(degree_index,d)
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

if __name__ == "__main__":
    run()
