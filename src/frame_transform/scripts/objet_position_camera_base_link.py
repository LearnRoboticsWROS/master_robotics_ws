#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from frame_transform.srv import FrameTransform, FrameTransformResponse

class ConversionFrameServer(object):
    def __init__(self):
        self.frame_sub = rospy.Subscriber("object_position_camera_frame", Point, self.conversion_callback)
        self.server = rospy.Service("/get_position_base_link", FrameTransform, self.handle_conversion)
        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/base_link", "camera_link", rospy.Time(0), rospy.Duration(4.0))

    


    def conversion_callback(self, data):

        # convert coordinates from camera_link frame to base_link frame
        point_camera = PointStamped()
        point_camera.header.frame_id = "camera_link"
        point_camera.header.stamp = rospy.Time(0)
        point_camera.point.x = data.x
        point_camera.point.y = data.y
        point_camera.point.z = data.z

        self.point_base_link = self.listener.transformPoint('base_link', point_camera) 


    def handle_conversion(self, req):

        if req:
            res = FrameTransformResponse()
            res.x_base_link_frame = self.point_base_link.point.x
            res.y_base_link_frame = self.point_base_link.point.y
            res.z_base_link_frame = self.point_base_link.point.z

            rospy.loginfo("Object position in base_link frame: x = %f, y = %f, z = %f" %(self.point_base_link.point.x, self.point_base_link.point.y, self.point_base_link.point.z))
        return res


if __name__ == '__main__':
    rospy.init_node('frame_conversion_server', anonymous=True)
    conversion_server = ConversionFrameServer()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")