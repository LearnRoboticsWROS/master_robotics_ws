;; Auto-generated. Do not edit!


(when (boundp 'frame_transform::FrameTransform)
  (if (not (find-package "FRAME_TRANSFORM"))
    (make-package "FRAME_TRANSFORM"))
  (shadow 'FrameTransform (find-package "FRAME_TRANSFORM")))
(unless (find-package "FRAME_TRANSFORM::FRAMETRANSFORM")
  (make-package "FRAME_TRANSFORM::FRAMETRANSFORM"))
(unless (find-package "FRAME_TRANSFORM::FRAMETRANSFORMREQUEST")
  (make-package "FRAME_TRANSFORM::FRAMETRANSFORMREQUEST"))
(unless (find-package "FRAME_TRANSFORM::FRAMETRANSFORMRESPONSE")
  (make-package "FRAME_TRANSFORM::FRAMETRANSFORMRESPONSE"))

(in-package "ROS")





(defclass frame_transform::FrameTransformRequest
  :super ros::object
  :slots (_from_camera_to_base_link ))

(defmethod frame_transform::FrameTransformRequest
  (:init
   (&key
    ((:from_camera_to_base_link __from_camera_to_base_link) nil)
    )
   (send-super :init)
   (setq _from_camera_to_base_link __from_camera_to_base_link)
   self)
  (:from_camera_to_base_link
   (&optional (__from_camera_to_base_link :null))
   (if (not (eq __from_camera_to_base_link :null)) (setq _from_camera_to_base_link __from_camera_to_base_link)) _from_camera_to_base_link)
  (:serialization-length
   ()
   (+
    ;; bool _from_camera_to_base_link
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _from_camera_to_base_link
       (if _from_camera_to_base_link (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _from_camera_to_base_link
     (setq _from_camera_to_base_link (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass frame_transform::FrameTransformResponse
  :super ros::object
  :slots (_x_base_link_frame _y_base_link_frame _z_base_link_frame ))

(defmethod frame_transform::FrameTransformResponse
  (:init
   (&key
    ((:x_base_link_frame __x_base_link_frame) 0.0)
    ((:y_base_link_frame __y_base_link_frame) 0.0)
    ((:z_base_link_frame __z_base_link_frame) 0.0)
    )
   (send-super :init)
   (setq _x_base_link_frame (float __x_base_link_frame))
   (setq _y_base_link_frame (float __y_base_link_frame))
   (setq _z_base_link_frame (float __z_base_link_frame))
   self)
  (:x_base_link_frame
   (&optional __x_base_link_frame)
   (if __x_base_link_frame (setq _x_base_link_frame __x_base_link_frame)) _x_base_link_frame)
  (:y_base_link_frame
   (&optional __y_base_link_frame)
   (if __y_base_link_frame (setq _y_base_link_frame __y_base_link_frame)) _y_base_link_frame)
  (:z_base_link_frame
   (&optional __z_base_link_frame)
   (if __z_base_link_frame (setq _z_base_link_frame __z_base_link_frame)) _z_base_link_frame)
  (:serialization-length
   ()
   (+
    ;; float64 _x_base_link_frame
    8
    ;; float64 _y_base_link_frame
    8
    ;; float64 _z_base_link_frame
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _x_base_link_frame
       (sys::poke _x_base_link_frame (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y_base_link_frame
       (sys::poke _y_base_link_frame (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _z_base_link_frame
       (sys::poke _z_base_link_frame (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _x_base_link_frame
     (setq _x_base_link_frame (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y_base_link_frame
     (setq _y_base_link_frame (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _z_base_link_frame
     (setq _z_base_link_frame (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass frame_transform::FrameTransform
  :super ros::object
  :slots ())

(setf (get frame_transform::FrameTransform :md5sum-) "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(setf (get frame_transform::FrameTransform :datatype-) "frame_transform/FrameTransform")
(setf (get frame_transform::FrameTransform :request) frame_transform::FrameTransformRequest)
(setf (get frame_transform::FrameTransform :response) frame_transform::FrameTransformResponse)

(defmethod frame_transform::FrameTransformRequest
  (:response () (instance frame_transform::FrameTransformResponse :init)))

(setf (get frame_transform::FrameTransformRequest :md5sum-) "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(setf (get frame_transform::FrameTransformRequest :datatype-) "frame_transform/FrameTransformRequest")
(setf (get frame_transform::FrameTransformRequest :definition-)
      "bool from_camera_to_base_link
---
float64 x_base_link_frame
float64 y_base_link_frame
float64 z_base_link_frame
")

(setf (get frame_transform::FrameTransformResponse :md5sum-) "1f0c6508a5e0cc15dc2cd25abbd47d9c")
(setf (get frame_transform::FrameTransformResponse :datatype-) "frame_transform/FrameTransformResponse")
(setf (get frame_transform::FrameTransformResponse :definition-)
      "bool from_camera_to_base_link
---
float64 x_base_link_frame
float64 y_base_link_frame
float64 z_base_link_frame
")



(provide :frame_transform/FrameTransform "1f0c6508a5e0cc15dc2cd25abbd47d9c")


