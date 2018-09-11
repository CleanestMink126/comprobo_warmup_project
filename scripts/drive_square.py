#For keyboard-controlled movement with AWSD and z
#Use AWSD to move into desired position and z to make 1x1 m square
#Comp Robo 2018 Warm-Up Project
from geometry_msgs.msg import PointStamped
import rospy
from teleop import *


def run():
    """Runs teleop and square command"""
    mytelC = TeleopC() #using teleop.py bc of similar use case
    while mytelC.key != '\x03':
        mytelC.key = mytelC.getKey() #checks for key press
        if mytelC.key == "a": #turn left
            mytelC.myspeedctrl.send_speed(0,1)
        elif mytelC.key =="s": #go backwards
            mytelC.myspeedctrl.send_speed(-1,0)
        elif mytelC.key == "w": #go forwards
            mytelC.myspeedctrl.send_speed(1,0)
        elif mytelC.key == "d": #turn right
            mytelC.myspeedctrl.send_speed(0,-1)
        elif mytelC.key == ' ': #stop
            mytelC.myspeedctrl.send_speed(0,0)
        elif mytelC.key == 'z': #turn in square
            for i in range(4):
                mytelC.myspeedctrl.send_speed(1,0)
                rospy.sleep(4) #value found by trail and error
                mytelC.myspeedctrl.send_speed(0,-1)
                rospy.sleep(1.61) #value found by trail and error

            mytelC.myspeedctrl.send_speed(0,0)


if __name__ == "__main__":
    run()
