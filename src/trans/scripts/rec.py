#! /usr/bin/env python
import rospy
from darknet_ros_msgs.msg import BoundingBoxes

def do_msg(msg):
    for box in msg.bounding_boxes:
        rospy.loginfo("%d %d %d %d %s", box.xmin, box.ymin, box.xmax, box.ymax, box.Class)

if __name__ == "__main__":
    rospy.init_node("rec")
    sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, do_msg, queue_size=10)
    rospy.spin()
    