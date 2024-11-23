; Auto-generated. Do not edit!


(cl:in-package frame_transform-srv)


;//! \htmlinclude FrameTransform-request.msg.html

(cl:defclass <FrameTransform-request> (roslisp-msg-protocol:ros-message)
  ((from_camera_to_base_link
    :reader from_camera_to_base_link
    :initarg :from_camera_to_base_link
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass FrameTransform-request (<FrameTransform-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrameTransform-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrameTransform-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name frame_transform-srv:<FrameTransform-request> is deprecated: use frame_transform-srv:FrameTransform-request instead.")))

(cl:ensure-generic-function 'from_camera_to_base_link-val :lambda-list '(m))
(cl:defmethod from_camera_to_base_link-val ((m <FrameTransform-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_transform-srv:from_camera_to_base_link-val is deprecated.  Use frame_transform-srv:from_camera_to_base_link instead.")
  (from_camera_to_base_link m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrameTransform-request>) ostream)
  "Serializes a message object of type '<FrameTransform-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'from_camera_to_base_link) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrameTransform-request>) istream)
  "Deserializes a message object of type '<FrameTransform-request>"
    (cl:setf (cl:slot-value msg 'from_camera_to_base_link) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrameTransform-request>)))
  "Returns string type for a service object of type '<FrameTransform-request>"
  "frame_transform/FrameTransformRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrameTransform-request)))
  "Returns string type for a service object of type 'FrameTransform-request"
  "frame_transform/FrameTransformRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrameTransform-request>)))
  "Returns md5sum for a message object of type '<FrameTransform-request>"
  "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrameTransform-request)))
  "Returns md5sum for a message object of type 'FrameTransform-request"
  "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrameTransform-request>)))
  "Returns full string definition for message of type '<FrameTransform-request>"
  (cl:format cl:nil "bool from_camera_to_base_link~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrameTransform-request)))
  "Returns full string definition for message of type 'FrameTransform-request"
  (cl:format cl:nil "bool from_camera_to_base_link~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrameTransform-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrameTransform-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FrameTransform-request
    (cl:cons ':from_camera_to_base_link (from_camera_to_base_link msg))
))
;//! \htmlinclude FrameTransform-response.msg.html

(cl:defclass <FrameTransform-response> (roslisp-msg-protocol:ros-message)
  ((x_base_link_frame
    :reader x_base_link_frame
    :initarg :x_base_link_frame
    :type cl:float
    :initform 0.0)
   (y_base_link_frame
    :reader y_base_link_frame
    :initarg :y_base_link_frame
    :type cl:float
    :initform 0.0)
   (z_base_link_frame
    :reader z_base_link_frame
    :initarg :z_base_link_frame
    :type cl:float
    :initform 0.0))
)

(cl:defclass FrameTransform-response (<FrameTransform-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FrameTransform-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FrameTransform-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name frame_transform-srv:<FrameTransform-response> is deprecated: use frame_transform-srv:FrameTransform-response instead.")))

(cl:ensure-generic-function 'x_base_link_frame-val :lambda-list '(m))
(cl:defmethod x_base_link_frame-val ((m <FrameTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_transform-srv:x_base_link_frame-val is deprecated.  Use frame_transform-srv:x_base_link_frame instead.")
  (x_base_link_frame m))

(cl:ensure-generic-function 'y_base_link_frame-val :lambda-list '(m))
(cl:defmethod y_base_link_frame-val ((m <FrameTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_transform-srv:y_base_link_frame-val is deprecated.  Use frame_transform-srv:y_base_link_frame instead.")
  (y_base_link_frame m))

(cl:ensure-generic-function 'z_base_link_frame-val :lambda-list '(m))
(cl:defmethod z_base_link_frame-val ((m <FrameTransform-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader frame_transform-srv:z_base_link_frame-val is deprecated.  Use frame_transform-srv:z_base_link_frame instead.")
  (z_base_link_frame m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FrameTransform-response>) ostream)
  "Serializes a message object of type '<FrameTransform-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'x_base_link_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'y_base_link_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'z_base_link_frame))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FrameTransform-response>) istream)
  "Deserializes a message object of type '<FrameTransform-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x_base_link_frame) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y_base_link_frame) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z_base_link_frame) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FrameTransform-response>)))
  "Returns string type for a service object of type '<FrameTransform-response>"
  "frame_transform/FrameTransformResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrameTransform-response)))
  "Returns string type for a service object of type 'FrameTransform-response"
  "frame_transform/FrameTransformResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FrameTransform-response>)))
  "Returns md5sum for a message object of type '<FrameTransform-response>"
  "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FrameTransform-response)))
  "Returns md5sum for a message object of type 'FrameTransform-response"
  "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FrameTransform-response>)))
  "Returns full string definition for message of type '<FrameTransform-response>"
  (cl:format cl:nil "float64 x_base_link_frame~%float64 y_base_link_frame~%float64 z_base_link_frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FrameTransform-response)))
  "Returns full string definition for message of type 'FrameTransform-response"
  (cl:format cl:nil "float64 x_base_link_frame~%float64 y_base_link_frame~%float64 z_base_link_frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FrameTransform-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FrameTransform-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FrameTransform-response
    (cl:cons ':x_base_link_frame (x_base_link_frame msg))
    (cl:cons ':y_base_link_frame (y_base_link_frame msg))
    (cl:cons ':z_base_link_frame (z_base_link_frame msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FrameTransform)))
  'FrameTransform-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FrameTransform)))
  'FrameTransform-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FrameTransform)))
  "Returns string type for a service object of type '<FrameTransform>"
  "frame_transform/FrameTransform")