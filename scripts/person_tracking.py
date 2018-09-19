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
        # self.distances = [] #list of distances to each point
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        self.my_marker = interface.SendLineMarker()
        x = None
        self.center_person = None
        while x is None:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
        self.thres = .25
        # self.distance = 1
        self.speed = .5
        self.track_pos = None
        self.unit_circle = np.array([[cos(theta),sin(theta)] for theta in np.linspace(0,2*pi,361)])

    def add_point(self,point):
        if not len(self.movements):
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            # self.distances.append(distance(point, (x,y)))
            self.movements.append(point)
        else:
            # self.distances.append(distance(self.movements[-1], point))
            self.movements.append(point)

    def navigate_to_point(self):
        self.my_speed.send_speed(0,0)
        x, y, yaw = self.my_lidar.my_odom.get_odom()
        while distance(self.movements[0],(x,y)) < self.thres:
            self.movements.pop(0)
            print('Popped Point')
            # self.distances.pop(0)
        angle_threshold = pi / 38
        c_angle_diff = angle_threshold + 1
        while abs(c_angle_diff) > angle_threshold:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            # print('yaw', yaw)
            x_t, y_t = self.movements[0][0], self.movements[0][1]
            t_yaw = atan2(y_t - y, x_t - x)
            c_angle_diff = angle_diff(t_yaw, yaw)
            self.my_speed.send_speed(.1-abs(c_angle_diff/(10*pi)), np.sign(c_angle_diff) * .5 +  c_angle_diff/2)
        self.my_speed.send_speed(self.speed,0)

    def look_for_person(self):
        value_range = 45
        difference_thres = .2
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        if (not len(list_ranges)):
            return False
        if (len(list_ranges) != len(list_odom)):
            print('NOOOOO not working')
            return False
        list_ranges= np.array(list_ranges[-1])
        odom = list_odom[-1]
        # print(list_ranges)
        # print(list_ranges)
        list_right = list_ranges[-value_range:-1]
        list_left = list_ranges[0:value_range]
        indices_right = np.where(np.bitwise_and(0<list_right, list_right<2))[0] + 360 - value_range
        indices_left = np.where(np.bitwise_and(0<list_left, list_left<2))[0]
        indices = np.concatenate([indices_left, indices_right])
        # print(indices)
        # print(indices)
        if not len(indices):
            return False
        # print(indices)
        # print(np.cos(indices * degrees2rad - odom[2]))
        x_mean = np.mean(np.cos(indices * degrees2rad + odom[2]) * list_ranges[indices] + odom[0])
        y_mean = np.mean(np.sin(indices * degrees2rad + odom[2]) * list_ranges[indices] + odom[1])
        if self.center_person is None:
            self.center_person = (x_mean, y_mean)
            return False
        if distance(self.center_person, (x_mean, y_mean)) > difference_thres:
            self.center_person = (x_mean, y_mean)
            return True
        self.center_person = (x_mean, y_mean)
        return False

    def append_new_points(self):
        #Do the math to add points onto movements
        diff_thres = .5
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        if not len(list_ranges):
            return None
        if (len(list_ranges) != len(list_odom)):
            print('NOOOOO not working')
            return None
        for list_range, odom in zip(list_ranges, list_odom):
            indices = np.where(list_range)[0]
            list_range = np.array(list_range)
            if not len(indices):
                return
            # print(indices)
            x_vals = np.cos(indices * degrees2rad + odom[2]) * list_range[indices] + odom[0]  - self.center_person[0]
            y_vals = np.sin(indices * degrees2rad + odom[2]) * list_range[indices] + odom[1]  - self.center_person[1]
            # print(x_vals)
            distances = np.linalg.norm(np.concatenate([np.expand_dims(x_vals,axis=-1),
                                np.expand_dims(y_vals,axis=-1)], axis = 1), axis = 1)
            distance_indices = np.argsort(distances)
            # print(distances[distance_indices])
            final_index = 0
            for i in range(len(distances)-1):
                final_index = i
                if distances[distance_indices[i+1]]- distances[distance_indices[i]] > diff_thres:
                    print(distances[distance_indices[i]] - distances[distance_indices[i+1]])
                    break
                if final_index > 8 or distances[distance_indices[i+1]] > 1:
                    break
            print(final_index)
            if final_index == 0:
                return None
            x_mean = np.mean(x_vals[distance_indices[:final_index]]) + self.center_person[0]
            y_mean = np.mean(y_vals[distance_indices[:final_index]]) + self.center_person[1]
            self.center_person = (x_mean,y_mean)
            self.add_point((x_mean,y_mean))
        print('-----------')
        for val in self.movements:
            print(val[0],val[1])

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
    def convert_to_avg(self):
        x, y = zip(*self.movements)
        x_conv = np.convolve([.125,.125,.125,.125,.125,.125,.125,.125], np.array(x), 'valid')
        y_conv = np.convolve([.125,.125,.125,.125,.125,.125,.125,.125], np.array(y), 'valid')
        return zip(x_conv, y_conv)

    def run(self):
        while not rospy.is_shutdown():
            if self.look_for_person():
                break
        while not rospy.is_shutdown():
            self.append_new_points()
            self.my_marker.update_marker(self.movements, frame_id = 'odom')
            if len(self.movements)  > 80:
                break
        self.movements = self.convert_to_avg()
        self.navigate_to_point()
        while not rospy.is_shutdown():
            while self.check_progress():
                    self.navigate_to_point()

        # self.append_new_points():

if __name__ == "__main__":
    print('Start')
    tracker = TrackOne()
    # tracker.add_point((0,0))
    # tracker.add_point((.5,0))#forward
    # tracker.add_point((0,.5))#to the left
    # # tracker.add_point(())
    # tracker.add_point((0,0))
    # print(tracker.movements)
    tracker.run()
