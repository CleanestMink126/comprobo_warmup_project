""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
import rospy
import teleop.TeleopC


def run():
    mytelC = TeleopC()
    while key != '\x03':
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
            r = rospy.Rate(.25)
            w = rospy.Rate(.25)
            for i in range(4):
                if i != 0:
                    mytelC.myspeedctrl.send_speed(0,1)
                    w.sleep()
                mytelC.myspeedctrl.send_speed(1,0)
                r.sleep()

if __name__ == "__main__":
    run()
