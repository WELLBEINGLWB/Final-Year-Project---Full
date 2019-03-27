; Auto-generated. Do not edit!


(cl:in-package segmentation-srv)


;//! \htmlinclude gazeOptimiser-request.msg.html

(cl:defclass <gazeOptimiser-request> (roslisp-msg-protocol:ros-message)
  ((objects
    :reader objects
    :initarg :objects
    :type std_msgs-msg:Float32MultiArray
    :initform (cl:make-instance 'std_msgs-msg:Float32MultiArray))
   (gaze_point
    :reader gaze_point
    :initarg :gaze_point
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point)))
)

(cl:defclass gazeOptimiser-request (<gazeOptimiser-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazeOptimiser-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazeOptimiser-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<gazeOptimiser-request> is deprecated: use segmentation-srv:gazeOptimiser-request instead.")))

(cl:ensure-generic-function 'objects-val :lambda-list '(m))
(cl:defmethod objects-val ((m <gazeOptimiser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:objects-val is deprecated.  Use segmentation-srv:objects instead.")
  (objects m))

(cl:ensure-generic-function 'gaze_point-val :lambda-list '(m))
(cl:defmethod gaze_point-val ((m <gazeOptimiser-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:gaze_point-val is deprecated.  Use segmentation-srv:gaze_point instead.")
  (gaze_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazeOptimiser-request>) ostream)
  "Serializes a message object of type '<gazeOptimiser-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objects) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'gaze_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazeOptimiser-request>) istream)
  "Deserializes a message object of type '<gazeOptimiser-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objects) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'gaze_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazeOptimiser-request>)))
  "Returns string type for a service object of type '<gazeOptimiser-request>"
  "segmentation/gazeOptimiserRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazeOptimiser-request)))
  "Returns string type for a service object of type 'gazeOptimiser-request"
  "segmentation/gazeOptimiserRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazeOptimiser-request>)))
  "Returns md5sum for a message object of type '<gazeOptimiser-request>"
  "8797234007ca66fde932e9a1f8ff3b3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazeOptimiser-request)))
  "Returns md5sum for a message object of type 'gazeOptimiser-request"
  "8797234007ca66fde932e9a1f8ff3b3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazeOptimiser-request>)))
  "Returns full string definition for message of type '<gazeOptimiser-request>"
  (cl:format cl:nil "std_msgs/Float32MultiArray objects~%geometry_msgs/Point gaze_point~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazeOptimiser-request)))
  "Returns full string definition for message of type 'gazeOptimiser-request"
  (cl:format cl:nil "std_msgs/Float32MultiArray objects~%geometry_msgs/Point gaze_point~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazeOptimiser-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objects))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'gaze_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazeOptimiser-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gazeOptimiser-request
    (cl:cons ':objects (objects msg))
    (cl:cons ':gaze_point (gaze_point msg))
))
;//! \htmlinclude gazeOptimiser-response.msg.html

(cl:defclass <gazeOptimiser-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass gazeOptimiser-response (<gazeOptimiser-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazeOptimiser-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazeOptimiser-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<gazeOptimiser-response> is deprecated: use segmentation-srv:gazeOptimiser-response instead.")))

(cl:ensure-generic-function 'sorted_objects-val :lambda-list '(m))
(cl:defmethod sorted_objects-val ((m <gazeOptimiser-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:sorted_objects-val is deprecated.  Use segmentation-srv:sorted_objects instead.")
  (sorted_objects m))

(cl:ensure-generic-function 'target_id-val :lambda-list '(m))
(cl:defmethod target_id-val ((m <gazeOptimiser-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:target_id-val is deprecated.  Use segmentation-srv:target_id instead.")
  (target_id m))

(cl:ensure-generic-function 'grasp_point-val :lambda-list '(m))
(cl:defmethod grasp_point-val ((m <gazeOptimiser-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:grasp_point-val is deprecated.  Use segmentation-srv:grasp_point instead.")
  (grasp_point m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazeOptimiser-response>) ostream)
  "Serializes a message object of type '<gazeOptimiser-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'sorted_objects) ostream)
  (cl:let* ((signed (cl:slot-value msg 'target_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grasp_point) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazeOptimiser-response>) istream)
  "Deserializes a message object of type '<gazeOptimiser-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'sorted_objects) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_id) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grasp_point) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazeOptimiser-response>)))
  "Returns string type for a service object of type '<gazeOptimiser-response>"
  "segmentation/gazeOptimiserResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazeOptimiser-response)))
  "Returns string type for a service object of type 'gazeOptimiser-response"
  "segmentation/gazeOptimiserResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazeOptimiser-response>)))
  "Returns md5sum for a message object of type '<gazeOptimiser-response>"
  "8797234007ca66fde932e9a1f8ff3b3b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazeOptimiser-response)))
  "Returns md5sum for a message object of type 'gazeOptimiser-response"
  "8797234007ca66fde932e9a1f8ff3b3b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazeOptimiser-response>)))
  "Returns full string definition for message of type '<gazeOptimiser-response>"
  (cl:format cl:nil "std_msgs/Float32MultiArray sorted_objects~%int16 target_id~%geometry_msgs/Point grasp_point~%~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazeOptimiser-response)))
  "Returns full string definition for message of type 'gazeOptimiser-response"
  (cl:format cl:nil "std_msgs/Float32MultiArray sorted_objects~%int16 target_id~%geometry_msgs/Point grasp_point~%~%~%================================================================================~%MSG: std_msgs/Float32MultiArray~%# Please look at the MultiArrayLayout message definition for~%# documentation on all multiarrays.~%~%MultiArrayLayout  layout        # specification of data layout~%float32[]         data          # array of data~%~%~%================================================================================~%MSG: std_msgs/MultiArrayLayout~%# The multiarray declares a generic multi-dimensional array of a~%# particular data type.  Dimensions are ordered from outer most~%# to inner most.~%~%MultiArrayDimension[] dim # Array of dimension properties~%uint32 data_offset        # padding elements at front of data~%~%# Accessors should ALWAYS be written in terms of dimension stride~%# and specified outer-most dimension first.~%# ~%# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]~%#~%# A standard, 3-channel 640x480 image with interleaved color channels~%# would be specified as:~%#~%# dim[0].label  = \"height\"~%# dim[0].size   = 480~%# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)~%# dim[1].label  = \"width\"~%# dim[1].size   = 640~%# dim[1].stride = 3*640 = 1920~%# dim[2].label  = \"channel\"~%# dim[2].size   = 3~%# dim[2].stride = 3~%#~%# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.~%~%================================================================================~%MSG: std_msgs/MultiArrayDimension~%string label   # label of given dimension~%uint32 size    # size of given dimension (in type units)~%uint32 stride  # stride of given dimension~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazeOptimiser-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'sorted_objects))
     2
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grasp_point))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazeOptimiser-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gazeOptimiser-response
    (cl:cons ':sorted_objects (sorted_objects msg))
    (cl:cons ':target_id (target_id msg))
    (cl:cons ':grasp_point (grasp_point msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gazeOptimiser)))
  'gazeOptimiser-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gazeOptimiser)))
  'gazeOptimiser-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazeOptimiser)))
  "Returns string type for a service object of type '<gazeOptimiser>"
  "segmentation/gazeOptimiser")