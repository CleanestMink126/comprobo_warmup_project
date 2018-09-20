import wall_following
import person_tracking
import teleop
import numpy as np

class FiniteState(object):
    def __init__(self):
        self.person_follower = person_tracking.TrackOne()
        self.controller = teleop.TeleopC()


    def run(self):
        while not rospy.is_shutdown():
            self.person_follower.run()
            degree_index = wall_following.run()
            self.controller.turn_90degrees(np.sign(degree_index - 180))
