#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np

class ObjectDetection(object):
    def __init__(self):
        self.image_sub = rospy.Subscriber("/cobot/camera1/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
    
    def camera_callback(self, data):
        try:
            cv_image  = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        # crop the image
        cropped_img = cv_image[100:350, 100:520]

        # convert image to grayscale
        gray_image = cv.cvtColor(cropped_img, cv.COLOR_BGR2GRAY)

        # create a mask based on threshold of grayscale
        mask_image = cv.adaptiveThreshold(gray_image, 255, cv.ADAPTIVE_THRESH_MEAN_C, cv.THRESH_BINARY_INV, 5, 15)

         # find contours
        contours, _ = cv.findContours(mask_image, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        print("contours: ", contours)

        for cnt in contours:
            cv.polylines(cropped_img, [cnt], True, [255, 0, 0], 1)

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
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.polylines(cropped_img, [box], True, (255, 0,0),1)
            cv.putText(cropped_img, "x: {}".format(round(x_center, 1)) + " y: {}".format(round(y_center,1)), (int(x_center), int(y_center)), cv.FONT_HERSHEY_PLAIN, 1, (0,255,0),1)
            cv.circle(cropped_img, (int(x_center), int(y_center)), 1, (255,0,0), thickness=-1)



        cv.imshow('original', cv_image)
        cv.imshow('cropped', cropped_img)
        cv.imshow('gray',gray_image)
        cv.imshow("mask",mask_image)
        cv.waitKey(1)


def main():
    object_detection = ObjectDetection()
    rospy.init_node('object_detection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down')
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()