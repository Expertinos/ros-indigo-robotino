; Auto-generated. Do not edit!


(cl:in-package robotino_local_move-msg)


;//! \htmlinclude PathDisplacements.msg.html

(cl:defclass <PathDisplacements> (roslisp-msg-protocol:ros-message)
  ((move_x
    :reader move_x
    :initarg :move_x
    :type cl:float
    :initform 0.0)
   (move_y
    :reader move_y
    :initarg :move_y
    :type cl:float
    :initform 0.0)
   (rotation
    :reader rotation
    :initarg :rotation
    :type cl:float
    :initform 0.0)
   (movementType
    :reader movementType
    :initarg :movementType
    :type cl:fixnum
    :initform 0))
)

(cl:defclass PathDisplacements (<PathDisplacements>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathDisplacements>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathDisplacements)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robotino_local_move-msg:<PathDisplacements> is deprecated: use robotino_local_move-msg:PathDisplacements instead.")))

(cl:ensure-generic-function 'move_x-val :lambda-list '(m))
(cl:defmethod move_x-val ((m <PathDisplacements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:move_x-val is deprecated.  Use robotino_local_move-msg:move_x instead.")
  (move_x m))

(cl:ensure-generic-function 'move_y-val :lambda-list '(m))
(cl:defmethod move_y-val ((m <PathDisplacements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:move_y-val is deprecated.  Use robotino_local_move-msg:move_y instead.")
  (move_y m))

(cl:ensure-generic-function 'rotation-val :lambda-list '(m))
(cl:defmethod rotation-val ((m <PathDisplacements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:rotation-val is deprecated.  Use robotino_local_move-msg:rotation instead.")
  (rotation m))

(cl:ensure-generic-function 'movementType-val :lambda-list '(m))
(cl:defmethod movementType-val ((m <PathDisplacements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robotino_local_move-msg:movementType-val is deprecated.  Use robotino_local_move-msg:movementType instead.")
  (movementType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathDisplacements>) ostream)
  "Serializes a message object of type '<PathDisplacements>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'move_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'move_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rotation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'movementType)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathDisplacements>) istream)
  "Deserializes a message object of type '<PathDisplacements>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'move_x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'move_y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'movementType)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathDisplacements>)))
  "Returns string type for a message object of type '<PathDisplacements>"
  "robotino_local_move/PathDisplacements")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathDisplacements)))
  "Returns string type for a message object of type 'PathDisplacements"
  "robotino_local_move/PathDisplacements")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathDisplacements>)))
  "Returns md5sum for a message object of type '<PathDisplacements>"
  "9b998c7d1cd4ec3c3324c335d8cef150")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathDisplacements)))
  "Returns md5sum for a message object of type 'PathDisplacements"
  "9b998c7d1cd4ec3c3324c335d8cef150")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathDisplacements>)))
  "Returns full string definition for message of type '<PathDisplacements>"
  (cl:format cl:nil "float32 move_x		# in meters~%float32 move_y		# in meters~%float32 rotation	# in degrees~%~%uint8 movementType	~%# 0 -> Translational Movement (move_phi should be equal to zero)~%# 1 -> Rotational Movement (move_x and move_y should be equal to zero)~%# 2 -> Translational and Rotational Moviment (at the same time)~%# 3 -> Tangent Moviment (move_x must be iqual to move_y)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathDisplacements)))
  "Returns full string definition for message of type 'PathDisplacements"
  (cl:format cl:nil "float32 move_x		# in meters~%float32 move_y		# in meters~%float32 rotation	# in degrees~%~%uint8 movementType	~%# 0 -> Translational Movement (move_phi should be equal to zero)~%# 1 -> Rotational Movement (move_x and move_y should be equal to zero)~%# 2 -> Translational and Rotational Moviment (at the same time)~%# 3 -> Tangent Moviment (move_x must be iqual to move_y)~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathDisplacements>))
  (cl:+ 0
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathDisplacements>))
  "Converts a ROS message object to a list"
  (cl:list 'PathDisplacements
    (cl:cons ':move_x (move_x msg))
    (cl:cons ':move_y (move_y msg))
    (cl:cons ':rotation (rotation msg))
    (cl:cons ':movementType (movementType msg))
))
