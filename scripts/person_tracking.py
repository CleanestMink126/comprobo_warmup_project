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
    '''This class will handle finding a person who moves directly in front of
    the neato, then tracking their position and following their path
    '''
    def __init__(self):
        self.movements = [] #list of tuples representing positions
        self.my_lidar = interface.BaseLidar()
        self.my_speed = interface.SendSpeed()
        self.my_marker = interface.SendLineMarker()
        self.center_person = None #track where the person currently is
        x = None
        while x is None:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
        self.thres = .25 #how close can a point be before the neato ignores it
        self.speed = .5 #how fast to move

    def add_point(self,point):
        '''Add a point to the list of points to follow'''
        if not len(self.movements):
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            self.movements.append(point)
        else:
            self.movements.append(point)

    def navigate_to_point(self):
        '''If we reached a point, turn to the next point in the list and
        drive there'''
        self.my_speed.send_speed(0,0)
        x, y, yaw = self.my_lidar.my_odom.get_odom()
        while distance(self.movements[0],(x,y)) < self.thres:
            self.movements.pop(0)#ignore redundant points that are close to neato
        angle_threshold = pi / 38 #how close we need to be in our heading to next point
        c_angle_diff = angle_threshold + 1 #just so the while loop will run
        while abs(c_angle_diff) > angle_threshold:
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            x_t, y_t = self.movements[0][0], self.movements[0][1]
            t_yaw = atan2(y_t - y, x_t - x) #calculate ideal angle
            c_angle_diff = angle_diff(t_yaw, yaw) #find difference from current angle
            #effectively turn but don't stop (with some hard coded numbers we found)
            self.my_speed.send_speed(.1-abs(c_angle_diff/(10*pi)), np.sign(c_angle_diff) * .5 +  c_angle_diff/2)
        self.my_speed.send_speed(self.speed,0)#go forward

    def look_for_person(self):
        '''The purpose of this function is to take the current laser scan and
        find points directly in front of the robot, so when the function is
        called again it can check if the center of these points has moved
        (i.e. there is a person in front)'''
        value_range = 45 #angle from center in which to loo
        difference_thres = .2 # how difference should the center be
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        if (not len(list_ranges)):
            return False
        if (len(list_ranges) != len(list_odom)):
            print('Lists not same size')
            return False
        list_ranges= np.array(list_ranges[-1]) #get most recent range
        odom = list_odom[-1]
        list_right = list_ranges[-value_range:-1]#random logic dealing with 0 -> 360 scan transition
        list_left = list_ranges[0:value_range]
        indices_right = np.where(np.bitwise_and(0<list_right, list_right<2))[0] + 360 - value_range
        indices_left = np.where(np.bitwise_and(0<list_left, list_left<2))[0]
        indices = np.concatenate([indices_left, indices_right]) #all indices in valid range
        if not len(indices):
            return False
        #Now we convert the can points to the right coordinate system
        x_mean = np.mean(np.cos(indices * degrees2rad + odom[2]) * list_ranges[indices] + odom[0])
        y_mean = np.mean(np.sin(indices * degrees2rad + odom[2]) * list_ranges[indices] + odom[1])
        if self.center_person is None: #for first measurement
            self.center_person = (x_mean, y_mean)
            return False
        if distance(self.center_person, (x_mean, y_mean)) > difference_thres:
            self.center_person = (x_mean, y_mean)
            return True
        self.center_person = (x_mean, y_mean)
        return False

    def append_new_points(self):
        '''Do the math to add points onto movements'''
        diff_thres = .5
        list_ranges, list_odom = self.my_lidar.get_list_ranges()
        if not len(list_ranges):#check for 0 length
            return None
        if (len(list_ranges) != len(list_odom)):
            print('Lists not same size')
            return None
        for list_range, odom in zip(list_ranges, list_odom):#for all observed ranges
            indices = np.where(list_range)[0] #get all non-zero indices
            list_range = np.array(list_range) #make the scan in numpy
            if not len(indices):
                return
            #convert from the neato-centered(base_frame) frame to non neato based(odom)
            #Then subtract out the center in question
            x_vals = np.cos(indices * degrees2rad + odom[2]) * list_range[indices] + odom[0]  - self.center_person[0]
            y_vals = np.sin(indices * degrees2rad + odom[2]) * list_range[indices] + odom[1]  - self.center_person[1]
            #find the distances between all points and the last observed center
            distances = np.linalg.norm(np.concatenate([np.expand_dims(x_vals,axis=-1),
                                np.expand_dims(y_vals,axis=-1)], axis = 1), axis = 1)
            distance_indices = np.argsort(distances)
            final_index = 0
            max_included = 8
            max_distance = .5
            #Now we work our way up from the closest points until there is a big
            #gap in distance,we exceed max_included points, or we are max_distance
            # units away from the last point
            for i in range(len(distances)-1):
                final_index = i
                if distances[distance_indices[i+1]]- distances[distance_indices[i]] > diff_thres:
                    break
                if final_index > max_included or distances[distance_indices[i+1]] > max_distance:
                    break
            print(final_index)
            if final_index == 0:
                continue
            x_mean = np.mean(x_vals[distance_indices[:final_index]]) + self.center_person[0]
            y_mean = np.mean(y_vals[distance_indices[:final_index]]) + self.center_person[1]
            self.center_person = (x_mean,y_mean)
            self.add_point((x_mean,y_mean))

    def check_progress(self):
        '''Loop to check if we have either passed our point or are close to it'''
        if len(self.movements):
            x, y, yaw = self.my_lidar.my_odom.get_odom()
            x_t, y_t = self.movements[0][0], self.movements[0][1]
            t_yaw = atan2(y_t - y, x_t - x)
            c_angle_diff = angle_diff(t_yaw, yaw)
            if distance(self.movements[0],(x,y)) < self.thres or abs(c_angle_diff)>pi/2:
                #We passed the point
                return True
        elif not len(self.movements):#stop if we run out of points
            self.my_speed.send_speed(0,0)
        return False

    def convert_to_avg(self):
        '''Take the current movements and pass a smoothing filter over them to
        take out some noise and make for better navigation'''
        x, y = zip(*self.movements)
        x_conv = np.convolve([.125,.125,.125,.125,.125,.125,.125,.125], np.array(x), 'valid')
        y_conv = np.convolve([.125,.125,.125,.125,.125,.125,.125,.125], np.array(y), 'valid')
        return zip(x_conv, y_conv)

    def run(self):
        '''Mainloop to control robot'''
        max_points = 80
        while not rospy.is_shutdown():#look until we find a person
            if self.look_for_person():
                break
        while not rospy.is_shutdown():#add the persons position for a set amount of time
            self.append_new_points()
            self.my_marker.update_marker(self.movements, frame_id = 'odom')
            if len(self.movements)  > max_points:
                break
        self.movements = self.convert_to_avg() #blurr points
        self.navigate_to_point() #got to first point
        # while not rospy.is_shutdown(): #loop to go through all points
        #     while self.check_progress() and len(self.movements):
        #             self.navigate_to_point()
        #             self.my_marker.update_marker(self.movements, frame_id = 'odom')

if __name__ == "__main__":
    print('Start')
    tracker = TrackOne()
    tracker.run()
