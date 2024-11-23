
(cl:in-package :asdf)

(defsystem "frame_transform-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "FrameTransform" :depends-on ("_package_FrameTransform"))
    (:file "_package_FrameTransform" :depends-on ("_package"))
  ))