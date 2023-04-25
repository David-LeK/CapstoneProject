; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude obj_msgs.msg.html

(cl:defclass <obj_msgs> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (northing
    :reader northing
    :initarg :northing
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (easting
    :reader easting
    :initarg :easting
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass obj_msgs (<obj_msgs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obj_msgs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obj_msgs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<obj_msgs> is deprecated: use custom_msg-msg:obj_msgs instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <obj_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:distance-val is deprecated.  Use custom_msg-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'northing-val :lambda-list '(m))
(cl:defmethod northing-val ((m <obj_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:northing-val is deprecated.  Use custom_msg-msg:northing instead.")
  (northing m))

(cl:ensure-generic-function 'easting-val :lambda-list '(m))
(cl:defmethod easting-val ((m <obj_msgs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:easting-val is deprecated.  Use custom_msg-msg:easting instead.")
  (easting m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obj_msgs>) ostream)
  "Serializes a message object of type '<obj_msgs>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'distance))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'northing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'northing))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'easting))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'easting))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obj_msgs>) istream)
  "Deserializes a message object of type '<obj_msgs>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'distance) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'distance)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'northing) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'northing)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'easting) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'easting)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obj_msgs>)))
  "Returns string type for a message object of type '<obj_msgs>"
  "custom_msg/obj_msgs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obj_msgs)))
  "Returns string type for a message object of type 'obj_msgs"
  "custom_msg/obj_msgs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obj_msgs>)))
  "Returns md5sum for a message object of type '<obj_msgs>"
  "2a1bea06901aaaf6ec5c025d3a77a953")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obj_msgs)))
  "Returns md5sum for a message object of type 'obj_msgs"
  "2a1bea06901aaaf6ec5c025d3a77a953")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obj_msgs>)))
  "Returns full string definition for message of type '<obj_msgs>"
  (cl:format cl:nil "float32[] distance~%float32[] northing~%float32[] easting~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obj_msgs)))
  "Returns full string definition for message of type 'obj_msgs"
  (cl:format cl:nil "float32[] distance~%float32[] northing~%float32[] easting~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obj_msgs>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'distance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'northing) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'easting) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obj_msgs>))
  "Converts a ROS message object to a list"
  (cl:list 'obj_msgs
    (cl:cons ':distance (distance msg))
    (cl:cons ':northing (northing msg))
    (cl:cons ':easting (easting msg))
))
