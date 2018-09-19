#For making Neato follow avoid obstacles
#Comp Robo 2018 Warm-Up Project

import numpy as np
import interface
import teleop
import rospy


def run(distance = 1.5, margin = .25):
    mytelC = teleop.TeleopC()
    mylidar = interface.ReceiveLidar()
    base_s = .25 #base speed
    base_t = 1 # base turning rate
    prop = 100.0 #proportion to turn (higher = less turning)
    turned = 0 #flag for if turned to avoid already or not
    d2 = 100

    while not rospy.is_shutdown():
        rospy.sleep(.5) #limits sampling rate
        degree_index, d = mylidar.get_wall(required_points=10)

        if degree_index != None: #if valid data received, determine what to do next
            print(degree_index,d)
            if degree_index <90 or degree_index >270: #object in front of Neato
                if d < distance + margin: #close distance state
                    if degree_index < 90:
                        mytelC.turn_90degrees('left')
                        turned = 1
                    elif degree_index > 270:
                        mytelC.turn_90degrees('right')
                        turned = 2
                    degree_index2, d2 = mylidar.get_wall(required_points=10)
                    while d2 < distance + margin:
                        if degree_index2  or degree_index2 :
                            mytelC.myspeedctrl.send_speed(base_s,0)
                            rospy.sleep(.25)
                            degree_index2, d2 = mylidar.get_wall(required_points=10)
                            print('degree2, d2', end='')
                            print(degree_index2, d2)
                    if turned == 1:
                        mytelC.turn_90degrees('right')
                    elif turned == 2:
                        mytelC.turn_90degrees('left')
                turned = 0
            else: #clear path state
                mytelC.myspeedctrl.send_speed(base_s,0)

        else:
            mytelC.myspeedctrl.send_speed(base_s,0) #if no wall found, go straight
    mytelC.myspeedctrl.send_speed(0,0) #when shut down, stop Neato from moving



if __name__ == "__main__":
    run()
