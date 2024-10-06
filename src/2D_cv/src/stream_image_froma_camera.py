#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class WriteImage(object):

    def __init__(self):
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self, data):
        try:
            cv_image  = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
            
        cv2.imshow('image', cv_image)
        cv2.waitKey(1)

        

def main():
    write_image_object = WriteImage()
    rospy.init_node('write_image_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()