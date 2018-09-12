import numpy as np
import interface
from math import sin, cos, pi

def distance(x,y):
    return np.linalg.norm(np.array(x)-np.array(y))

rad2degrees = 180/pi

class TrackOne(object):
    def __init__(self):
        self.movements = [] #list of tuples representing positions
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        self.thres = .05
        self.distance = 1
        self.track_pos = None
        self.unit_circle = np.array([[cos(theta),sin(theta)] for theta in np.linspace(0,2*pi,361)])

    def navigate_to_point(self):
        #figure out how much and long to turn, then go forward
        #TODO

    def append_new_points(self):
        #Do the math to add points onto movements
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        new_ranges = [np.roll(c_range,int(c_odom[3] * rad2degrees)) for c_range, c_odom in zip(list_ranges,list_odom)]
        final_points = [self.unit_circle * np.tile(np.expand_dims(c_range,-1),2) - [c_odom[0], c_odom[1]] for c_range, c_odom in zip(new_ranges,list_odom)]
        #add in filtering earlier
        #TODO

    def check_progress(self):
        if len(self.movements) and distance(self.movements[0],self.my_odom.get_pos()) < self.threshold:
            return True
        return False

    def check_stop(self):
        #check if the current progress would put the robot within the person
        #stop if so
        #start if not
        #TODO

    def run(self):
        #we need to navigate to the next spot while looking for new spots
        while self.check_progress():
            self.navigate_to_point()

        self.append_new_points():
