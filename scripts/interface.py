""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
from sensor_msgs import LaserScan
from neato_node import Bump
from neato_node import Accel
from geometry_msgs import Twist
from geometry_msgs.Vector3 import linear
from geometry_msgs.Vector3 import angular
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

    def send_speed(move_forward = 0, turn_left = 0):
        my_point_stamped = Twist(linear=linear(move_forward,0,0), angular=angular(0,0,turn_left))
        self.publisher.publish(my_point_stamped)


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
