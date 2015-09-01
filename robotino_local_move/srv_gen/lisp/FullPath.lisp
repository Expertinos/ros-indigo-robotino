; Auto-generated. Do not edit!


(cl:in-package robotino_local_move-srv)


;//! \htmlinclude FullPath-request.msg.html

(cl:defclass <FullPath-request> (roslisp-msg-protocol:ros-message)
  ((goal
    :reader goal
    :initarg :goal
    :type cl:integer
    :initform 0))
)

(cl:defclass FullPath-request (<FullPath-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullPath-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullPath-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_local_move-srv:<FullPath-request> is deprecated: use robotino_local_move-srv:FullPath-request instead.")))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <FullPath-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-srv:goal-val is deprecated.  Use robotino_local_move-srv:goal instead.")
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullPath-request>) ostream)
  "Serializes a message object of type '<FullPath-request>"
  (cl:let* ((signed (cl:slot-value msg 'goal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullPath-request>) istream)
  "Deserializes a message object of type '<FullPath-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goal) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullPath-request>)))
  "Returns string type for a service object of type '<FullPath-request>"
  "robotino_local_move/FullPathRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullPath-request)))
  "Returns string type for a service object of type 'FullPath-request"
  "robotino_local_move/FullPathRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullPath-request>)))
  "Returns md5sum for a message object of type '<FullPath-request>"
  "e27a1c3698284a9b9f545d211e208fd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullPath-request)))
  "Returns md5sum for a message object of type 'FullPath-request"
  "e27a1c3698284a9b9f545d211e208fd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullPath-request>)))
  "Returns full string definition for message of type '<FullPath-request>"
  (cl:format cl:nil "int32 goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullPath-request)))
  "Returns full string definition for message of type 'FullPath-request"
  (cl:format cl:nil "int32 goal~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullPath-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullPath-request>))
  "Converts a ROS message object to a list"
  (cl:list 'FullPath-request
    (cl:cons ':goal (goal msg))
))
;//! \htmlinclude FullPath-response.msg.html

(cl:defclass <FullPath-response> (roslisp-msg-protocol:ros-message)
  ((full_path
    :reader full_path
    :initarg :full_path
    :type cl:string
    :initform ""))
)

(cl:defclass FullPath-response (<FullPath-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FullPath-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FullPath-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_local_move-srv:<FullPath-response> is deprecated: use robotino_local_move-srv:FullPath-response instead.")))

(cl:ensure-generic-function 'full_path-val :lambda-list '(m))
(cl:defmethod full_path-val ((m <FullPath-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-srv:full_path-val is deprecated.  Use robotino_local_move-srv:full_path instead.")
  (full_path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FullPath-response>) ostream)
  "Serializes a message object of type '<FullPath-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'full_path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'full_path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FullPath-response>) istream)
  "Deserializes a message object of type '<FullPath-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'full_path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'full_path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FullPath-response>)))
  "Returns string type for a service object of type '<FullPath-response>"
  "robotino_local_move/FullPathResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullPath-response)))
  "Returns string type for a service object of type 'FullPath-response"
  "robotino_local_move/FullPathResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FullPath-response>)))
  "Returns md5sum for a message object of type '<FullPath-response>"
  "e27a1c3698284a9b9f545d211e208fd5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FullPath-response)))
  "Returns md5sum for a message object of type 'FullPath-response"
  "e27a1c3698284a9b9f545d211e208fd5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FullPath-response>)))
  "Returns full string definition for message of type '<FullPath-response>"
  (cl:format cl:nil "string full_path~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FullPath-response)))
  "Returns full string definition for message of type 'FullPath-response"
  (cl:format cl:nil "string full_path~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FullPath-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'full_path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FullPath-response>))
  "Converts a ROS message object to a list"
  (cl:list 'FullPath-response
    (cl:cons ':full_path (full_path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'FullPath)))
  'FullPath-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'FullPath)))
  'FullPath-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FullPath)))
  "Returns string type for a service object of type '<FullPath>"
  "robotino_local_move/FullPath")