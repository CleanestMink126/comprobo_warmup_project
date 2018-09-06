""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
from sensor_msgs import LaserScan
import rospy

class ReceiveMessageode(object):
    def __init__(self):
        rospy.init_node('receive_message')
        rospy.Subscriber("/my_point", PointStamped, process_point)


    def process_point(self, m):
        print(m)

    def run(self):
        rospy.spin()

class ReceiveLidar(object):
    def __init__(self):
        rospy.init_node('receive_lidar')
        rospy.Subscriber("/my_point", LaserScan, process_point)

    def process_range(self, m):
        return m.range

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ReceiveMessageNode()
    node.run()
