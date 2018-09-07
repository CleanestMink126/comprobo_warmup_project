""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
import rospy
from teleop import *


def run():
    mytelC = TeleopC()
    while mytelC.key != '\x03':
        mytelC.key = mytelC.getKey()
        if mytelC.key == "a":
            mytelC.myspeedctrl.send_speed(0,1)
        elif mytelC.key =="s":
            mytelC.myspeedctrl.send_speed(-1,0)
        elif mytelC.key == "w":
            mytelC.myspeedctrl.send_speed(1,0)
        elif mytelC.key == "d":
            mytelC.myspeedctrl.send_speed(0,-1)
        elif mytelC.key == ' ':
            mytelC.myspeedctrl.send_speed(0,0)
        elif mytelC.key == 'z':
            r = rospy.Rate(1)
            w = rospy.Rate(1)
            for i in range(4):
                mytelC.myspeedctrl.send_speed(1,0)
                print('before r')
                r.sleep()
                print('after r')
                mytelC.myspeedctrl.send_speed(0,-1)
                print('before w')
                w.sleep()
                print('after w')

            mytelC.myspeedctrl.send_speed(0,0)


if __name__ == "__main__":
    run()
