; Auto-generated. Do not edit!


(cl:in-package segmentation-srv)


;//! \htmlinclude gazePoint-request.msg.html

(cl:defclass <gazePoint-request> (roslisp-msg-protocol:ros-message)
  ((gaze_x
    :reader gaze_x
    :initarg :gaze_x
    :type cl:float
    :initform 0.0)
   (gaze_y
    :reader gaze_y
    :initarg :gaze_y
    :type cl:float
    :initform 0.0)
   (gaze_z
    :reader gaze_z
    :initarg :gaze_z
    :type cl:float
    :initform 0.0))
)

(cl:defclass gazePoint-request (<gazePoint-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazePoint-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazePoint-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<gazePoint-request> is deprecated: use segmentation-srv:gazePoint-request instead.")))

(cl:ensure-generic-function 'gaze_x-val :lambda-list '(m))
(cl:defmethod gaze_x-val ((m <gazePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:gaze_x-val is deprecated.  Use segmentation-srv:gaze_x instead.")
  (gaze_x m))

(cl:ensure-generic-function 'gaze_y-val :lambda-list '(m))
(cl:defmethod gaze_y-val ((m <gazePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:gaze_y-val is deprecated.  Use segmentation-srv:gaze_y instead.")
  (gaze_y m))

(cl:ensure-generic-function 'gaze_z-val :lambda-list '(m))
(cl:defmethod gaze_z-val ((m <gazePoint-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:gaze_z-val is deprecated.  Use segmentation-srv:gaze_z instead.")
  (gaze_z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazePoint-request>) ostream)
  "Serializes a message object of type '<gazePoint-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gaze_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gaze_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'gaze_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazePoint-request>) istream)
  "Deserializes a message object of type '<gazePoint-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gaze_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gaze_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'gaze_z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazePoint-request>)))
  "Returns string type for a service object of type '<gazePoint-request>"
  "segmentation/gazePointRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazePoint-request)))
  "Returns string type for a service object of type 'gazePoint-request"
  "segmentation/gazePointRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazePoint-request>)))
  "Returns md5sum for a message object of type '<gazePoint-request>"
  "775e18c60bb3bbf62cc8777df14f5d02")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazePoint-request)))
  "Returns md5sum for a message object of type 'gazePoint-request"
  "775e18c60bb3bbf62cc8777df14f5d02")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazePoint-request>)))
  "Returns full string definition for message of type '<gazePoint-request>"
  (cl:format cl:nil "float32 gaze_x~%float32 gaze_y~%float32 gaze_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazePoint-request)))
  "Returns full string definition for message of type 'gazePoint-request"
  (cl:format cl:nil "float32 gaze_x~%float32 gaze_y~%float32 gaze_z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazePoint-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazePoint-request>))
  "Converts a ROS message object to a list"
  (cl:list 'gazePoint-request
    (cl:cons ':gaze_x (gaze_x msg))
    (cl:cons ':gaze_y (gaze_y msg))
    (cl:cons ':gaze_z (gaze_z msg))
))
;//! \htmlinclude gazePoint-response.msg.html

(cl:defclass <gazePoint-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass gazePoint-response (<gazePoint-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gazePoint-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gazePoint-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<gazePoint-response> is deprecated: use segmentation-srv:gazePoint-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <gazePoint-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:success-val is deprecated.  Use segmentation-srv:success instead.")
  (success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gazePoint-response>) ostream)
  "Serializes a message object of type '<gazePoint-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gazePoint-response>) istream)
  "Deserializes a message object of type '<gazePoint-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gazePoint-response>)))
  "Returns string type for a service object of type '<gazePoint-response>"
  "segmentation/gazePointResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazePoint-response)))
  "Returns string type for a service object of type 'gazePoint-response"
  "segmentation/gazePointResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gazePoint-response>)))
  "Returns md5sum for a message object of type '<gazePoint-response>"
  "775e18c60bb3bbf62cc8777df14f5d02")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gazePoint-response)))
  "Returns md5sum for a message object of type 'gazePoint-response"
  "775e18c60bb3bbf62cc8777df14f5d02")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gazePoint-response>)))
  "Returns full string definition for message of type '<gazePoint-response>"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gazePoint-response)))
  "Returns full string definition for message of type 'gazePoint-response"
  (cl:format cl:nil "bool success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gazePoint-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gazePoint-response>))
  "Converts a ROS message object to a list"
  (cl:list 'gazePoint-response
    (cl:cons ':success (success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'gazePoint)))
  'gazePoint-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'gazePoint)))
  'gazePoint-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gazePoint)))
  "Returns string type for a service object of type '<gazePoint>"
  "segmentation/gazePoint")