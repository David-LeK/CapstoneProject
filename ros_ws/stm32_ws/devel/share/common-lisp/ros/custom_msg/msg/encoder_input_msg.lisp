; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude encoder_input_msg.msg.html

(cl:defclass <encoder_input_msg> (roslisp-msg-protocol:ros-message)
  ((input_setpoint_m1
    :reader input_setpoint_m1
    :initarg :input_setpoint_m1
    :type cl:float
    :initform 0.0)
   (input_Kp_m1
    :reader input_Kp_m1
    :initarg :input_Kp_m1
    :type cl:float
    :initform 0.0)
   (input_Ki_m1
    :reader input_Ki_m1
    :initarg :input_Ki_m1
    :type cl:float
    :initform 0.0)
   (input_Kd_m1
    :reader input_Kd_m1
    :initarg :input_Kd_m1
    :type cl:float
    :initform 0.0)
   (input_setpoint_m2
    :reader input_setpoint_m2
    :initarg :input_setpoint_m2
    :type cl:float
    :initform 0.0)
   (input_Kp_m2
    :reader input_Kp_m2
    :initarg :input_Kp_m2
    :type cl:float
    :initform 0.0)
   (input_Ki_m2
    :reader input_Ki_m2
    :initarg :input_Ki_m2
    :type cl:float
    :initform 0.0)
   (input_Kd_m2
    :reader input_Kd_m2
    :initarg :input_Kd_m2
    :type cl:float
    :initform 0.0))
)

(cl:defclass encoder_input_msg (<encoder_input_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <encoder_input_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'encoder_input_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<encoder_input_msg> is deprecated: use custom_msg-msg:encoder_input_msg instead.")))

(cl:ensure-generic-function 'input_setpoint_m1-val :lambda-list '(m))
(cl:defmethod input_setpoint_m1-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_setpoint_m1-val is deprecated.  Use custom_msg-msg:input_setpoint_m1 instead.")
  (input_setpoint_m1 m))

(cl:ensure-generic-function 'input_Kp_m1-val :lambda-list '(m))
(cl:defmethod input_Kp_m1-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_Kp_m1-val is deprecated.  Use custom_msg-msg:input_Kp_m1 instead.")
  (input_Kp_m1 m))

(cl:ensure-generic-function 'input_Ki_m1-val :lambda-list '(m))
(cl:defmethod input_Ki_m1-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_Ki_m1-val is deprecated.  Use custom_msg-msg:input_Ki_m1 instead.")
  (input_Ki_m1 m))

(cl:ensure-generic-function 'input_Kd_m1-val :lambda-list '(m))
(cl:defmethod input_Kd_m1-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_Kd_m1-val is deprecated.  Use custom_msg-msg:input_Kd_m1 instead.")
  (input_Kd_m1 m))

(cl:ensure-generic-function 'input_setpoint_m2-val :lambda-list '(m))
(cl:defmethod input_setpoint_m2-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_setpoint_m2-val is deprecated.  Use custom_msg-msg:input_setpoint_m2 instead.")
  (input_setpoint_m2 m))

(cl:ensure-generic-function 'input_Kp_m2-val :lambda-list '(m))
(cl:defmethod input_Kp_m2-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_Kp_m2-val is deprecated.  Use custom_msg-msg:input_Kp_m2 instead.")
  (input_Kp_m2 m))

(cl:ensure-generic-function 'input_Ki_m2-val :lambda-list '(m))
(cl:defmethod input_Ki_m2-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_Ki_m2-val is deprecated.  Use custom_msg-msg:input_Ki_m2 instead.")
  (input_Ki_m2 m))

(cl:ensure-generic-function 'input_Kd_m2-val :lambda-list '(m))
(cl:defmethod input_Kd_m2-val ((m <encoder_input_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:input_Kd_m2-val is deprecated.  Use custom_msg-msg:input_Kd_m2 instead.")
  (input_Kd_m2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <encoder_input_msg>) ostream)
  "Serializes a message object of type '<encoder_input_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_setpoint_m1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_Kp_m1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_Ki_m1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_Kd_m1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_setpoint_m2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_Kp_m2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_Ki_m2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'input_Kd_m2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <encoder_input_msg>) istream)
  "Deserializes a message object of type '<encoder_input_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_setpoint_m1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Kp_m1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Ki_m1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Kd_m1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_setpoint_m2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Kp_m2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Ki_m2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'input_Kd_m2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<encoder_input_msg>)))
  "Returns string type for a message object of type '<encoder_input_msg>"
  "custom_msg/encoder_input_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'encoder_input_msg)))
  "Returns string type for a message object of type 'encoder_input_msg"
  "custom_msg/encoder_input_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<encoder_input_msg>)))
  "Returns md5sum for a message object of type '<encoder_input_msg>"
  "db9ba92c90b9ba885220db60c812fbcf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'encoder_input_msg)))
  "Returns md5sum for a message object of type 'encoder_input_msg"
  "db9ba92c90b9ba885220db60c812fbcf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<encoder_input_msg>)))
  "Returns full string definition for message of type '<encoder_input_msg>"
  (cl:format cl:nil "float32 input_setpoint_m1~%float32 input_Kp_m1~%float32 input_Ki_m1~%float32 input_Kd_m1~%float32 input_setpoint_m2~%float32 input_Kp_m2~%float32 input_Ki_m2~%float32 input_Kd_m2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'encoder_input_msg)))
  "Returns full string definition for message of type 'encoder_input_msg"
  (cl:format cl:nil "float32 input_setpoint_m1~%float32 input_Kp_m1~%float32 input_Ki_m1~%float32 input_Kd_m1~%float32 input_setpoint_m2~%float32 input_Kp_m2~%float32 input_Ki_m2~%float32 input_Kd_m2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <encoder_input_msg>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <encoder_input_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'encoder_input_msg
    (cl:cons ':input_setpoint_m1 (input_setpoint_m1 msg))
    (cl:cons ':input_Kp_m1 (input_Kp_m1 msg))
    (cl:cons ':input_Ki_m1 (input_Ki_m1 msg))
    (cl:cons ':input_Kd_m1 (input_Kd_m1 msg))
    (cl:cons ':input_setpoint_m2 (input_setpoint_m2 msg))
    (cl:cons ':input_Kp_m2 (input_Kp_m2 msg))
    (cl:cons ':input_Ki_m2 (input_Ki_m2 msg))
    (cl:cons ':input_Kd_m2 (input_Kd_m2 msg))
))
