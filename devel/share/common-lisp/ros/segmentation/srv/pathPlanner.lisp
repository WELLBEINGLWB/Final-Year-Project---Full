; Auto-generated. Do not edit!


(cl:in-package segmentation-srv)


;//! \htmlinclude pathPlanner-request.msg.html

(cl:defclass <pathPlanner-request> (roslisp-msg-protocol:ros-message)
  ((sorted_objects
    :reader sorted_objects
    :initarg :sorted_objects
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (target_id
    :reader target_id
    :initarg :target_id
    :type cl:fixnum
    :initform 0)
   (grasp_point
    :reader grasp_point
    :initarg :grasp_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass pathPlanner-request (<pathPlanner-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pathPlanner-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pathPlanner-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<pathPlanner-request> is deprecated: use segmentation-srv:pathPlanner-request instead.")))

(cl:ensure-generic-function 'sorted_objects-val :lambda-list '(m))
(cl:defmethod sorted_objects-val ((m <pathPlanner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:sorted_objects-val is deprecated.  Use segmentation-srv:sorted_objects instead.")
  (sorted_objects m))

(cl:ensure-generic-function 'target_id-val :lambda-list '(m))
(cl:defmethod target_id-val ((m <pathPlanner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:target_id-val is deprecated.  Use segmentation-srv:target_id instead.")
  (target_id m))

(cl:ensure-generic-function 'grasp_point-val :lambda-list '(m))
(cl:defmethod grasp_point-val ((m <pathPlanner-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:grasp_point-val is deprecated.  Use segmentation-srv:grasp_point instead.")
  (grasp_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pathPlanner-request>) ostream)
  "Serializes a message object of type '<pathPlanner-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sorted_objects) ostream)
  (cl:let* ((signed (cl:slot-value msg 'target_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grasp_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pathPlanner-request>) istream)
  "Deserializes a message object of type '<pathPlanner-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sorted_objects) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grasp_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pathPlanner-request>)))
  "Returns string type for a service object of type '<pathPlanner-request>"
  "segmentation/pathPlannerRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pathPlanner-request)))
  "Returns string type for a service object of type 'pathPlanner-request"
  "segmentation/pathPlannerRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pathPlanner-request>)))
  "Returns md5sum for a message object of type '<pathPlanner-request>"
  "29b555fe93d25b5dcf4f386d833186f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pathPlanner-request)))
  "Returns md5sum for a message object of type 'pathPlanner-request"
  "29b555fe93d25b5dcf4f386d833186f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pathPlanner-request>)))
  "Returns full string definition for message of type '<pathPlanner-request>"
  (cl:format cl:nil "std_msgs/Float32MultiArray sorted_objects~%int16 target_id~%geometry_msgs/Point grasp_point~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pathPlanner-request)))
  "Returns full string definition for message of type 'pathPlanner-request"
  (cl:format cl:nil "std_msgs/Float32MultiArray sorted_objects~%int16 target_id~%geometry_msgs/Point grasp_point~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pathPlanner-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sorted_objects))
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grasp_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pathPlanner-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pathPlanner-request
    (cl:cons ':sorted_objects (sorted_objects msg))
    (cl:cons ':target_id (target_id msg))
    (cl:cons ':grasp_point (grasp_point msg))
))
;//! \htmlinclude pathPlanner-response.msg.html

(cl:defclass <pathPlanner-response> (roslisp-msg-protocol:ros-message)
  ((path_found
    :reader path_found
    :initarg :path_found
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass pathPlanner-response (<pathPlanner-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pathPlanner-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pathPlanner-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<pathPlanner-response> is deprecated: use segmentation-srv:pathPlanner-response instead.")))

(cl:ensure-generic-function 'path_found-val :lambda-list '(m))
(cl:defmethod path_found-val ((m <pathPlanner-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:path_found-val is deprecated.  Use segmentation-srv:path_found instead.")
  (path_found m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pathPlanner-response>) ostream)
  "Serializes a message object of type '<pathPlanner-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'path_found) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pathPlanner-response>) istream)
  "Deserializes a message object of type '<pathPlanner-response>"
    (cl:setf (cl:slot-value msg 'path_found) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pathPlanner-response>)))
  "Returns string type for a service object of type '<pathPlanner-response>"
  "segmentation/pathPlannerResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pathPlanner-response)))
  "Returns string type for a service object of type 'pathPlanner-response"
  "segmentation/pathPlannerResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pathPlanner-response>)))
  "Returns md5sum for a message object of type '<pathPlanner-response>"
  "29b555fe93d25b5dcf4f386d833186f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pathPlanner-response)))
  "Returns md5sum for a message object of type 'pathPlanner-response"
  "29b555fe93d25b5dcf4f386d833186f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pathPlanner-response>)))
  "Returns full string definition for message of type '<pathPlanner-response>"
  (cl:format cl:nil "bool path_found~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pathPlanner-response)))
  "Returns full string definition for message of type 'pathPlanner-response"
  (cl:format cl:nil "bool path_found~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pathPlanner-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pathPlanner-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pathPlanner-response
    (cl:cons ':path_found (path_found msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pathPlanner)))
  'pathPlanner-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pathPlanner)))
  'pathPlanner-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pathPlanner)))
  "Returns string type for a service object of type '<pathPlanner>"
  "segmentation/pathPlanner")