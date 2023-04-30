; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude encoder_output_msg.msg.html

(cl:defclass <encoder_output_msg> (roslisp-msg-protocol:ros-message)
  ((output_rpm_m1
    :reader output_rpm_m1
    :initarg :output_rpm_m1
    :type cl:float
    :initform 0.0)
   (output_controller_m1
    :reader output_controller_m1
    :initarg :output_controller_m1
    :type cl:float
    :initform 0.0)
   (output_rpm_m2
    :reader output_rpm_m2
    :initarg :output_rpm_m2
    :type cl:float
    :initform 0.0)
   (output_controller_m2
    :reader output_controller_m2
    :initarg :output_controller_m2
    :type cl:float
    :initform 0.0))
)

(cl:defclass encoder_output_msg (<encoder_output_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <encoder_output_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'encoder_output_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<encoder_output_msg> is deprecated: use custom_msg-msg:encoder_output_msg instead.")))

(cl:ensure-generic-function 'output_rpm_m1-val :lambda-list '(m))
(cl:defmethod output_rpm_m1-val ((m <encoder_output_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:output_rpm_m1-val is deprecated.  Use custom_msg-msg:output_rpm_m1 instead.")
  (output_rpm_m1 m))

(cl:ensure-generic-function 'output_controller_m1-val :lambda-list '(m))
(cl:defmethod output_controller_m1-val ((m <encoder_output_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:output_controller_m1-val is deprecated.  Use custom_msg-msg:output_controller_m1 instead.")
  (output_controller_m1 m))

(cl:ensure-generic-function 'output_rpm_m2-val :lambda-list '(m))
(cl:defmethod output_rpm_m2-val ((m <encoder_output_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:output_rpm_m2-val is deprecated.  Use custom_msg-msg:output_rpm_m2 instead.")
  (output_rpm_m2 m))

(cl:ensure-generic-function 'output_controller_m2-val :lambda-list '(m))
(cl:defmethod output_controller_m2-val ((m <encoder_output_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:output_controller_m2-val is deprecated.  Use custom_msg-msg:output_controller_m2 instead.")
  (output_controller_m2 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <encoder_output_msg>) ostream)
  "Serializes a message object of type '<encoder_output_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output_rpm_m1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output_controller_m1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output_rpm_m2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'output_controller_m2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <encoder_output_msg>) istream)
  "Deserializes a message object of type '<encoder_output_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output_rpm_m1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output_controller_m1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output_rpm_m2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'output_controller_m2) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<encoder_output_msg>)))
  "Returns string type for a message object of type '<encoder_output_msg>"
  "custom_msg/encoder_output_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'encoder_output_msg)))
  "Returns string type for a message object of type 'encoder_output_msg"
  "custom_msg/encoder_output_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<encoder_output_msg>)))
  "Returns md5sum for a message object of type '<encoder_output_msg>"
  "1245866a2cb06bf0708dba963cb3a6ee")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'encoder_output_msg)))
  "Returns md5sum for a message object of type 'encoder_output_msg"
  "1245866a2cb06bf0708dba963cb3a6ee")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<encoder_output_msg>)))
  "Returns full string definition for message of type '<encoder_output_msg>"
  (cl:format cl:nil "float32 output_rpm_m1~%float32 output_controller_m1~%float32 output_rpm_m2~%float32 output_controller_m2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'encoder_output_msg)))
  "Returns full string definition for message of type 'encoder_output_msg"
  (cl:format cl:nil "float32 output_rpm_m1~%float32 output_controller_m1~%float32 output_rpm_m2~%float32 output_controller_m2~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <encoder_output_msg>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <encoder_output_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'encoder_output_msg
    (cl:cons ':output_rpm_m1 (output_rpm_m1 msg))
    (cl:cons ':output_controller_m1 (output_controller_m1 msg))
    (cl:cons ':output_rpm_m2 (output_rpm_m2 msg))
    (cl:cons ':output_controller_m2 (output_controller_m2 msg))
))
