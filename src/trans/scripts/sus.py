#! /usr/bin/env python
import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import message_filters
import numpy as np
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
from darknet_ros_msgs.msg import BoundingBoxes

class ImageListener:
    def __init__(self, depth_image_topic, depth_info_topic):
        self.bridge = CvBridge()
        # self.sub = rospy.Subscriber(depth_image_topic, msg_Image, self.imageDepthCallback)
        # self.sub_info = rospy.Subscriber(depth_info_topic, CameraInfo, self.imageDepthInfoCallback)
        # self.sub_bbox = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.BoundingBoxesCallback, queue_size=10)
        # confidence_topic = depth_image_topic.replace('depth', 'confidence')
        # self.sub_conf = rospy.Subscriber(confidence_topic, msg_Image, self.confidenceCallback)
        self.sub = message_filters.Subscriber(depth_image_topic, msg_Image)
        self.sub_info = message_filters.Subscriber(depth_info_topic, CameraInfo)
        self.sub_bbox = message_filters.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.sub, self.sub_info, self.sub_bbox],
                                                                queue_size=10, slop=0.5, allow_headerless=True)
        self.ts.registerCallback(self.callback)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None

    def callback(self, data, cameraInfo, bboxes):
        try:
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

        cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        for box in bboxes.bounding_boxes:
            line = f"{box.Class} is within [{box.xmin} {box.ymin} {box.xmax} {box.ymax}] pixels\n"
            pix = ((box.xmin + box.xmax) // 2, (box.ymin + box.ymax) // 2)
            self.pix = pix
            # line += ' Depth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])
            depth = cv_image[pix[1], pix[0]]
            result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
            line += 'The corresponding real coordinates are: %8.2f %8.2f %8.2f.\n' % (result[0], result[1], result[2])
            # if self.intrinsics:
            #     depth = cv_image[pix[1], pix[0]]
            #     result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
            #     line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            # line += '\r'
            line += '\n\n'
            sys.stdout.write(line)
            sys.stdout.flush()

    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # pick one pixel among all the pixels with the closest range:
            indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
            pix = (indices[1], indices[0])
            pix = (320, 240)
            self.pix = pix
            line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

            if self.intrinsics:
                depth = cv_image[pix[1], pix[0]]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
                line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
            if (not self.pix_grade is None):
                line += ' Grade: %2d' % self.pix_grade
            line += '\r'
            sys.stdout.write(line)
            sys.stdout.flush()

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    # def confidenceCallback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    #         grades = np.bitwise_and(cv_image >> 4, 0x0f)
    #         if (self.pix):
    #             self.pix_grade = grades[self.pix[1], self.pix[0]]
    #     except CvBridgeError as e:
    #         print(e)
    #         return



    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.K[2]
            self.intrinsics.ppy = cameraInfo.K[5]
            self.intrinsics.fx = cameraInfo.K[0]
            self.intrinsics.fy = cameraInfo.K[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.D]
        except CvBridgeError as e:
            print(e)
            return

    def BoundingBoxesCallback(self, bboxes):
        for box in bboxes.bounding_boxes:
            self.xmin = box.xmin
            print(f"{box.xmin} {box.ymin} {box.xmax} {box.ymax} {box.Class}")


def main():
    depth_image_topic = '/camera/depth/image_rect_raw'
    depth_info_topic = '/camera/depth/camera_info'

    # print ('')
    # print ('show_center_depth.py')
    # print ('--------------------')
    # print ('App to demontrate the usage of the /camera/depth topics.')
    # print ('')
    # print ('Application subscribes to %s and %s topics.' % (depth_image_topic, depth_info_topic))
    # print ('Application then calculates and print the range to the closest object.')
    # print ('If intrinsics data is available, it also prints the 3D location of the object')
    # print ('If a confedence map is also available in the topic %s, it also prints the confidence grade.' % depth_image_topic.replace('depth', 'confidence'))
    # print ('')
    
    listener = ImageListener(depth_image_topic, depth_info_topic)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node("suc")
    main()