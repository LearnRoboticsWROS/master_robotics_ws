#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class ColorFilter(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image  = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        min_red = np.array([0,216, 250])
        max_red = np.array([180,255, 255])

        mask_r = cv.inRange(hsv_image, min_red, max_red)

        res_r = cv.bitwise_and(cv_image, cv_image, mask = mask_r)

         # find contours
        contours, _ = cv.findContours(mask_r, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        print("contours: ", contours)

        for cnt in contours:
            cv.polylines(cv_image, [cnt], True, [255, 0, 0], 1)

        object_detected = []
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > 20:
                cnt = cv.approxPolyDP(cnt, 0.03*cv.arcLength(cnt, True), True)
                object_detected.append(cnt)
        print("how many object I detect: ", len(object_detected))
        print(object_detected)

        for cnt in object_detected:
            rect = cv.minAreaRect(cnt)
            (x_center, y_center), (w,h), orientation = rect
            print("width: "+str(w)+" height: " +str(h))
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.polylines(cv_image, [box], True, (0, 255,0),1)
            cv.putText(cv_image, "x: {}".format(round(x_center, 1)) + " y: {}".format(round(y_center,1)), (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)
            cv.circle(cv_image, (int(x_center), int(y_center)), 1, (0,255,0), thickness=-1)



        cv.imshow('original', cv_image)
        cv.imshow('hsv', hsv_image)
        cv.imshow("mask on color",mask_r)
        cv.imshow("red on top", res_r)
        cv.waitKey(1)


def main():
    color_filter_object = ColorFilter()
    rospy.init_node('color_filter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()