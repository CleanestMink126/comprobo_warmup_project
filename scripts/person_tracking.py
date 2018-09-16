import numpy as np
import interface
from math import sin, cos, atan2, pi, fabs
import rospy
print('Imported')

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
degrees2rad = pi/180

class TrackOne(object):
    def __init__(self):
        self.movements = [] #list of tuples representing positions
        self.distances = [] #list of distances to each point
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        x = None
        self.center_peron = None
        while x is None:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
        self.thres = .05
        self.distance = 1
        self.speed = .5
        self.track_pos = None
        self.unit_circle = np.array([[cos(theta),sin(theta)] for theta in np.linspace(0,2*pi,361)])

    def add_point(self,point):
        if not len(self.movements):
            x, y, yaw = self.my_lidar.my_odom.get_odom()

            self.distances.append(distance(point, (x,y)))
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
        angle_threshold = pi / 38
        c_angle_diff = angle_threshold + 1
        while abs(c_angle_diff) > angle_threshold:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            print('yaw', yaw)
            x_t, y_t = self.movements[0][0], self.movements[0][1]
            t_yaw = atan2(y_t - y, x_t - x)
            c_angle_diff = angle_diff(t_yaw, yaw)
            self.my_speed.send_speed(0, np.sign(c_angle_diff) * .1 +  c_angle_diff/2)
        self.my_speed.send_speed(self.speed,0)

    def look_for person(self):
        value_range = 45
        difference_thres = .05
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        if not len(list_ranges):
            return False
        values = np.arange(361)
        indices_right = np.where(list_ranges[-1:-value_range:-1])[0]
        indices_left = np.where(list_ranges[0:value_range])[0]
        indices = np.concat([indices_left, indices_right]) * degrees2rad
        if not len(indices):
            return False
        x_mean = np.mean(np.cos(indices) * values[indices])
        y_mean = np.mean(np.sin(indices) * values[indices])
        if self.center_person is None:
            self.center_peron = (x_mean, y_mean)
            return False
         if distance(self.center_peron, (x-mean, y_mean)) > difference_thres:
             return True
        self.center_peron = (x_mean, y_mean)
        return False

    def append_new_points(self):
        #Do the math to add points onto movements
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        new_ranges = [np.roll(c_range,int(c_odom[3] * rad2degrees)) for c_range, c_odom in zip(list_ranges,list_odom)]
        final_points = [self.unit_circle * np.tile(np.expand_dims(c_range,-1),2) - [c_odom[0], c_odom[1]] for c_range, c_odom in zip(new_ranges,list_odom)]

        #add in filtering earlier
        #TODO

    def check_progress(self):
        if len(self.movements):
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            x_t, y_t = self.movements[0][0], self.movements[0][1]
            t_yaw = atan2(y_t - y, x_t - x)
            c_angle_diff = angle_diff(t_yaw, yaw)
            if distance(self.movements[0],(x,y)) < self.thres or abs(c_angle_diff)>pi/2:
                return True
        elif not len(self.movements):
            self.my_speed.send_speed(0,0)
        return False

    def check_stop(self):
        #check if the current progress would put the robot within the person
        #stop if so
        #start if not
        #TODO
        pass

    def run(self):
        #we need to navigate to the next spot while looking for new spots
        # self.navigate_to_point()
        # while not rospy.is_shutdown():
        #
        #     while self.check_progress():
        #         self.navigate_to_point()
        while not rospy.is_shutdown():
            if self.look_for_person():
                print('Found one!')
        # self.append_new_points():

if __name__ == "__main__":
    print('Start')
    tracker = TrackOne()
    tracker.add_point((0,0))
    tracker.add_point((.5,0))#forward
    tracker.add_point((0,.5))#to the left
    # tracker.add_point(())
    tracker.add_point((0,0))
    print(tracker.movements)
    tracker.run()
