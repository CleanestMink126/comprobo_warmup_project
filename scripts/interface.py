""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import LaserScan
from neato_node.msg import Bump
from neato_node.msg import Accel
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

import rospy

###############################################################################
#Sending classes
###############################################################################

class MessageNode(object):
    """This node publishes a message at 2 Hz"""
    def __init__(self):
        rospy.init_node('test_message')   #initialize with roscore
        self.publisher = rospy.Publisher('/my_point', PointStamped, queue_size=10)

    def run(self):
        r = rospy.Rate(2)  #Publishing at 2 Hz
        while not rospy.is_shutdown():
            my_point_stamped = PointStamped(header=Header(stamp=rospy.Time.now(), frame_id="odom"), point=Point(1.0,2.0,0.0))
            publisher.publish(my_point_stamped)
            r.sleep()

class SendSpeed(object):
    def __init__(self):
        rospy.init_node('Get_Speed')   #initialize with roscore
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def send_speed(self,move_forward = None, turn_left = None):
        if move_forward == None:
            move_forward = 0
        if turn_left == None:
            turn_left = 0
        my_point_stamped = Twist(linear=Vector3(move_forward,0,0), angular=Vector3(0,0,turn_left))
        self.publisher.publish(my_point_stamped)

    def low_rider(self):
        t = 10
        r = rospy.Rate(5)
        for i in range(t):
            my_point_stamped = Twist(linear=Vector3(1,0,0))
            self.publisher.publish(my_point_stamped)
            r.sleep()
            my_point_stamped = Twist(linear=Vector3(-1,0,0))
            self.publisher.publish(my_point_stamped)
            r.sleep()

###############################################################################
#Receiving classes
###############################################################################

class ReceiveLidar(object):
    def __init__(self):
        rospy.init_node('receive_lidar')
        rospy.Subscriber("/scan", LaserScan, process_point)

    def process_range(self, m):
        print(m.range)

    def run(self):
        rospy.spin()

class ReceiveBump(object):
    def __init__(self):
        rospy.init_node('receive_lidar')
        rospy.Subscriber("/bump", Bump, process_bump)

    def process_bump(self, m):
        return m

    def run(self):
        rospy.spin()

class ReceiveAccel(object):
    def __init__(self):
        rospy.init_node('receive_lidar')
        rospy.Subscriber("/accel", Accel, process_accel)

    def process_accel(self, m):
        return m

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ReceiveMessageNode()
    node.run()
