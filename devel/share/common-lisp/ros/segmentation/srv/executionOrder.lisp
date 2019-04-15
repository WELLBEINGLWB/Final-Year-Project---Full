; Auto-generated. Do not edit!


(cl:in-package segmentation-srv)


;//! \htmlinclude executionOrder-request.msg.html

(cl:defclass <executionOrder-request> (roslisp-msg-protocol:ros-message)
  ((execute_order
    :reader execute_order
    :initarg :execute_order
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass executionOrder-request (<executionOrder-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <executionOrder-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'executionOrder-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<executionOrder-request> is deprecated: use segmentation-srv:executionOrder-request instead.")))

(cl:ensure-generic-function 'execute_order-val :lambda-list '(m))
(cl:defmethod execute_order-val ((m <executionOrder-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:execute_order-val is deprecated.  Use segmentation-srv:execute_order instead.")
  (execute_order m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <executionOrder-request>) ostream)
  "Serializes a message object of type '<executionOrder-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execute_order) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <executionOrder-request>) istream)
  "Deserializes a message object of type '<executionOrder-request>"
    (cl:setf (cl:slot-value msg 'execute_order) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<executionOrder-request>)))
  "Returns string type for a service object of type '<executionOrder-request>"
  "segmentation/executionOrderRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'executionOrder-request)))
  "Returns string type for a service object of type 'executionOrder-request"
  "segmentation/executionOrderRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<executionOrder-request>)))
  "Returns md5sum for a message object of type '<executionOrder-request>"
  "79dbb0357db7f2c6aa3e8bc9c33c69eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'executionOrder-request)))
  "Returns md5sum for a message object of type 'executionOrder-request"
  "79dbb0357db7f2c6aa3e8bc9c33c69eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<executionOrder-request>)))
  "Returns full string definition for message of type '<executionOrder-request>"
  (cl:format cl:nil "bool execute_order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'executionOrder-request)))
  "Returns full string definition for message of type 'executionOrder-request"
  (cl:format cl:nil "bool execute_order~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <executionOrder-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <executionOrder-request>))
  "Converts a ROS message object to a list"
  (cl:list 'executionOrder-request
    (cl:cons ':execute_order (execute_order msg))
))
;//! \htmlinclude executionOrder-response.msg.html

(cl:defclass <executionOrder-response> (roslisp-msg-protocol:ros-message)
  ((execution_success
    :reader execution_success
    :initarg :execution_success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass executionOrder-response (<executionOrder-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <executionOrder-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'executionOrder-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name segmentation-srv:<executionOrder-response> is deprecated: use segmentation-srv:executionOrder-response instead.")))

(cl:ensure-generic-function 'execution_success-val :lambda-list '(m))
(cl:defmethod execution_success-val ((m <executionOrder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader segmentation-srv:execution_success-val is deprecated.  Use segmentation-srv:execution_success instead.")
  (execution_success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <executionOrder-response>) ostream)
  "Serializes a message object of type '<executionOrder-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'execution_success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <executionOrder-response>) istream)
  "Deserializes a message object of type '<executionOrder-response>"
    (cl:setf (cl:slot-value msg 'execution_success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<executionOrder-response>)))
  "Returns string type for a service object of type '<executionOrder-response>"
  "segmentation/executionOrderResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'executionOrder-response)))
  "Returns string type for a service object of type 'executionOrder-response"
  "segmentation/executionOrderResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<executionOrder-response>)))
  "Returns md5sum for a message object of type '<executionOrder-response>"
  "79dbb0357db7f2c6aa3e8bc9c33c69eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'executionOrder-response)))
  "Returns md5sum for a message object of type 'executionOrder-response"
  "79dbb0357db7f2c6aa3e8bc9c33c69eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<executionOrder-response>)))
  "Returns full string definition for message of type '<executionOrder-response>"
  (cl:format cl:nil "bool execution_success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'executionOrder-response)))
  "Returns full string definition for message of type 'executionOrder-response"
  (cl:format cl:nil "bool execution_success~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <executionOrder-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <executionOrder-response>))
  "Converts a ROS message object to a list"
  (cl:list 'executionOrder-response
    (cl:cons ':execution_success (execution_success msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'executionOrder)))
  'executionOrder-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'executionOrder)))
  'executionOrder-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'executionOrder)))
  "Returns string type for a service object of type '<executionOrder>"
  "segmentation/executionOrder")