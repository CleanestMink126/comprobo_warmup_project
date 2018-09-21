import wall_following
import person_tracking
import teleop
import numpy as np
import rospy
class FiniteState(object):
    def __init__(self):
        self.person_follower = person_tracking.TrackOne()
        self.controller = teleop.TeleopC()


    def run(self):
        while not rospy.is_shutdown():
            degree_index = wall_following.run()
            if degree_index < 180:
                self.controller.turn_90degrees('left')
            else:
                self.controller.turn_90degrees('right')
            self.person_follower.run()

if __name__ == '__main__':
    m = FiniteState()
    m.run()
