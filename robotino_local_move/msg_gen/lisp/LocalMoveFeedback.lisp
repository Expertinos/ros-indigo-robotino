; Auto-generated. Do not edit!


(cl:in-package robotino_local_move-msg)


;//! \htmlinclude LocalMoveFeedback.msg.html

(cl:defclass <LocalMoveFeedback> (roslisp-msg-protocol:ros-message)
  ((forward_dist_x
    :reader forward_dist_x
    :initarg :forward_dist_x
    :type cl:float
    :initform 0.0)
   (forward_dist_y
    :reader forward_dist_y
    :initarg :forward_dist_y
    :type cl:float
    :initform 0.0)
   (rotation_dist
    :reader rotation_dist
    :initarg :rotation_dist
    :type cl:float
    :initform 0.0))
)

(cl:defclass LocalMoveFeedback (<LocalMoveFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LocalMoveFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LocalMoveFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_local_move-msg:<LocalMoveFeedback> is deprecated: use robotino_local_move-msg:LocalMoveFeedback instead.")))

(cl:ensure-generic-function 'forward_dist_x-val :lambda-list '(m))
(cl:defmethod forward_dist_x-val ((m <LocalMoveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:forward_dist_x-val is deprecated.  Use robotino_local_move-msg:forward_dist_x instead.")
  (forward_dist_x m))

(cl:ensure-generic-function 'forward_dist_y-val :lambda-list '(m))
(cl:defmethod forward_dist_y-val ((m <LocalMoveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:forward_dist_y-val is deprecated.  Use robotino_local_move-msg:forward_dist_y instead.")
  (forward_dist_y m))

(cl:ensure-generic-function 'rotation_dist-val :lambda-list '(m))
(cl:defmethod rotation_dist-val ((m <LocalMoveFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:rotation_dist-val is deprecated.  Use robotino_local_move-msg:rotation_dist instead.")
  (rotation_dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LocalMoveFeedback>) ostream)
  "Serializes a message object of type '<LocalMoveFeedback>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forward_dist_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'forward_dist_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotation_dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LocalMoveFeedback>) istream)
  "Deserializes a message object of type '<LocalMoveFeedback>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forward_dist_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'forward_dist_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation_dist) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LocalMoveFeedback>)))
  "Returns string type for a message object of type '<LocalMoveFeedback>"
  "robotino_local_move/LocalMoveFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LocalMoveFeedback)))
  "Returns string type for a message object of type 'LocalMoveFeedback"
  "robotino_local_move/LocalMoveFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LocalMoveFeedback>)))
  "Returns md5sum for a message object of type '<LocalMoveFeedback>"
  "fcda879d7916aea8cf95cd23201a7d19")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LocalMoveFeedback)))
  "Returns md5sum for a message object of type 'LocalMoveFeedback"
  "fcda879d7916aea8cf95cd23201a7d19")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LocalMoveFeedback>)))
  "Returns full string definition for message of type '<LocalMoveFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float32 forward_dist_x	# forward distance to goal in x~%float32 forward_dist_y	# forward distance to goal in y~%float32 rotation_dist	# rotationaldistance to goal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LocalMoveFeedback)))
  "Returns full string definition for message of type 'LocalMoveFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float32 forward_dist_x	# forward distance to goal in x~%float32 forward_dist_y	# forward distance to goal in y~%float32 rotation_dist	# rotationaldistance to goal~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LocalMoveFeedback>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LocalMoveFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'LocalMoveFeedback
    (cl:cons ':forward_dist_x (forward_dist_x msg))
    (cl:cons ':forward_dist_y (forward_dist_y msg))
    (cl:cons ':rotation_dist (rotation_dist msg))
))
