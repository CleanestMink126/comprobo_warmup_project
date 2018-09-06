""" Investigate receiving a message using a callback function """

from geometry_msgs.msg import PointStamped
import rospy

class ReceiveMessageode(object):
    def __init__(self):
        rospy.init_node('receive_message')
        rospy.Subscriber("/my_point", PointStamped, process_point)


    def process_point(self, m):
        print(m)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ReceiveMessageNode()
    node.run()
