#For keyboard-controlled movement with AWSD and l
#Use AWSD to move into desired position and l to move back and forth rapidly
#Comp Robo 2018 Warm-Up Project


import tty
import select
import sys
import termios
import interface
import rospy


class TeleopC(object):
    '''Controller for neato
    Uses WASD controls with space to stop the neato'''
    def __init__ (self):
        self.settings = termios.tcgetattr(sys.stdin)
        self.myspeedctrl = interface.SendSpeed()
        self.key = None

    def getKey(self):
        '''Given code to grab keyboard values'''
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        self.key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return self.key

    def run(self):
        '''Run this loop to control the robot.
        ^C to quite.'''
        while self.key != '\x03':
            self.key = self.getKey()
            if self.key == "a": #turn left
                self.myspeedctrl.send_speed(0,1)
            elif self.key =="s": #go backwards
                self.myspeedctrl.send_speed(-1,0)
            elif self.key == "w": #go forwards
                self.myspeedctrl.send_speed(1,0)
            elif self.key == "d": #turn right
                self.myspeedctrl.send_speed(0,-1)
            elif self.key == ' ': #stop
                self.myspeedctrl.send_speed(0,0)
            elif self.key == 'l': #activate "low-rider" mode
                self.myspeedctrl.low_rider()

    def turn_90degrees(self, direction):
        """  Turns Neato by 90 degrees based on direction ('right'/1 or 'left'/2)
        Values found by trial and error, not odom
        Works for turning speed 1 and sleep value 1.61"""
        if direction == "right" or direction == 1:
            self.myspeedctrl.send_speed(0,1)
        elif direction == "left" or direction == 2:
            self.myspeedctrl.send_speed(0,-1)
        rospy.sleep(1.61) #value found by trail and error
        self.myspeedctrl.send_speed(0,0)


if __name__ == "__main__":
    myTeleop = TeleopC()
    myTeleop.run()
