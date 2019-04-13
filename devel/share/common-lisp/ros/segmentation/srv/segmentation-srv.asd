
(cl:in-package :asdf)

(defsystem "segmentation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "gazeOptimiser" :depends-on ("_package_gazeOptimiser"))
    (:file "_package_gazeOptimiser" :depends-on ("_package"))
    (:file "pathPlanner" :depends-on ("_package_pathPlanner"))
    (:file "_package_pathPlanner" :depends-on ("_package"))
    (:file "seg" :depends-on ("_package_seg"))
    (:file "_package_seg" :depends-on ("_package"))
    (:file "gazePoint" :depends-on ("_package_gazePoint"))
    (:file "_package_gazePoint" :depends-on ("_package"))
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
  ))