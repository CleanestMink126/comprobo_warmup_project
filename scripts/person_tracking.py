import numpy as np
import interface
from math import sin, cos, atan2, pi, fabs

def distance(x,y):
    return np.linalg.norm(np.array(x)-np.array(y))

def angle_normalize(z):
    """ convenience function to map an angle to the range [-pi,pi] """
    return atan2(sin(z), cos(z))

def angle_diff(a, b):
    """ Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    """
    a = angle_normalize(a)
    b = angle_normalize(b)
    d1 = a-b
    d2 = 2*pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2

rad2degrees = 180/pi

class TrackOne(object):
    def __init__(self):
        self.movements = [] #list of tuples representing positions
        self.distances = [] #list of distances to each point
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        self.thres = .05
        self.distance = 1
        self.speed = .5
        self.track_pos = None
        self.unit_circle = np.array([[cos(theta),sin(theta)] for theta in np.linspace(0,2*pi,361)])

    def add_point(point):
        if not len(self.movements):
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            self.distances.append(distance(self.movements[-1], (x,y)))
            self.movements.append(point)
        else:
            self.distances.append(distance(self.movements[-1], point))
            self.movements.append(point)



    def navigate_to_point(self):
        self.my_speed.send_speed(0,0)
        x, y, yaw = self.my_lidar.my_odom.get_odom()
        while distance(self.movements[0],(x,y)) < self.thres:
            self.movements.pop(0)
            self.distances.pop(0)
        angle_threshold = pi / 18
        angle_diff = angle_threshold + 1
        while abs(angle_diff) > angle_threshold:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            x_t, y_t = self.movements[0,0], self.movements[0,1]
            t_yaw = atan2(y_t - y, x_t - x)
            angle_diff = angle_diff(yaw, t_yaw)
            self.my_speed.send_speed(0, angle_diff/2)
        self.my_speed.send_speed(self.speed,0)

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
        elif not len(self.movements):
            self.my_speed.send_speed(0,0)
        return False

    def check_stop(self):
        #check if the current progress would put the robot within the person
        #stop if so
        #start if not
        #TODO

    def run(self):
        #we need to navigate to the next spot while looking for new spots
        while not rospy.is_shutdown():
            while self.check_progress():
                self.navigate_to_point()

        # self.append_new_points():

if __name__ == "__main__":
    tracker = TrackOne()
    tracker.add_point((1,2))
    tracker.add_point((-2,2))
    tracker.add_point((1,-2))
    tracker.add_point((0,0))
    tracker.run()
