; Auto-generated. Do not edit!


(cl:in-package robotino_local_move-msg)


;//! \htmlinclude PathStatus.msg.html

(cl:defclass <PathStatus> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PathStatus (<PathStatus>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathStatus>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathStatus)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_local_move-msg:<PathStatus> is deprecated: use robotino_local_move-msg:PathStatus instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <PathStatus>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:status-val is deprecated.  Use robotino_local_move-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathStatus>) ostream)
  "Serializes a message object of type '<PathStatus>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathStatus>) istream)
  "Deserializes a message object of type '<PathStatus>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'status)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathStatus>)))
  "Returns string type for a message object of type '<PathStatus>"
  "robotino_local_move/PathStatus")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathStatus)))
  "Returns string type for a message object of type 'PathStatus"
  "robotino_local_move/PathStatus")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathStatus>)))
  "Returns md5sum for a message object of type '<PathStatus>"
  "284aa12dd9e9e760802ac9f38036ea5e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathStatus)))
  "Returns md5sum for a message object of type 'PathStatus"
  "284aa12dd9e9e760802ac9f38036ea5e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathStatus>)))
  "Returns full string definition for message of type '<PathStatus>"
  (cl:format cl:nil "uint8 status~%# 0 -> Aborted~%# 1 -> Completed~%# 2 -> Timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathStatus)))
  "Returns full string definition for message of type 'PathStatus"
  (cl:format cl:nil "uint8 status~%# 0 -> Aborted~%# 1 -> Completed~%# 2 -> Timeout~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathStatus>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathStatus>))
  "Converts a ROS message object to a list"
  (cl:list 'PathStatus
    (cl:cons ':status (status msg))
))
