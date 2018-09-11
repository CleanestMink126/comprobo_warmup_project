import numpy as np
import interface

def distance(x,y):
    return np.linalg.norm(np.array(x)-np.array(y))

class TrackOne(object):

    def __init__(self):
        self.movements = [] #list of tuples representing positions
        self.my_odom = interface.ReceiveOdom()
        self.my_lidar = interface.ReceiveLidar()
        self.my_speed = interface.SendSpeed()
        self.thres = .05

    def navigate_to_point(self):
        #TODO
    def append_new_points(self):
        #TODO
        
    def check_progress(self):
        if len(self.movements) and distance(self.movements[0],self.my_odom.get_pos()) < self.threshold:
            return True
        return False

    def run(self):
        #we need to navigate to the next spot while looking for new spots
        if self.check_progress():
            self.navigate_to_point()

        self.append_new_points():
