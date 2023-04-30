; Auto-generated. Do not edit!


(cl:in-package custom_msg-msg)


;//! \htmlinclude gps_msg.msg.html

(cl:defclass <gps_msg> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (speed_kmh
    :reader speed_kmh
    :initarg :speed_kmh
    :type cl:float
    :initform 0.0)
   (northing
    :reader northing
    :initarg :northing
    :type cl:float
    :initform 0.0)
   (easting
    :reader easting
    :initarg :easting
    :type cl:float
    :initform 0.0)
   (tracking_angle
    :reader tracking_angle
    :initarg :tracking_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass gps_msg (<gps_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gps_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gps_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name custom_msg-msg:<gps_msg> is deprecated: use custom_msg-msg:gps_msg instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <gps_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:latitude-val is deprecated.  Use custom_msg-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <gps_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:longitude-val is deprecated.  Use custom_msg-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'speed_kmh-val :lambda-list '(m))
(cl:defmethod speed_kmh-val ((m <gps_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:speed_kmh-val is deprecated.  Use custom_msg-msg:speed_kmh instead.")
  (speed_kmh m))

(cl:ensure-generic-function 'northing-val :lambda-list '(m))
(cl:defmethod northing-val ((m <gps_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:northing-val is deprecated.  Use custom_msg-msg:northing instead.")
  (northing m))

(cl:ensure-generic-function 'easting-val :lambda-list '(m))
(cl:defmethod easting-val ((m <gps_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:easting-val is deprecated.  Use custom_msg-msg:easting instead.")
  (easting m))

(cl:ensure-generic-function 'tracking_angle-val :lambda-list '(m))
(cl:defmethod tracking_angle-val ((m <gps_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader custom_msg-msg:tracking_angle-val is deprecated.  Use custom_msg-msg:tracking_angle instead.")
  (tracking_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gps_msg>) ostream)
  "Serializes a message object of type '<gps_msg>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_kmh))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'northing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'easting))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'tracking_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gps_msg>) istream)
  "Deserializes a message object of type '<gps_msg>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_kmh) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'northing) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'easting) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'tracking_angle) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gps_msg>)))
  "Returns string type for a message object of type '<gps_msg>"
  "custom_msg/gps_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gps_msg)))
  "Returns string type for a message object of type 'gps_msg"
  "custom_msg/gps_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gps_msg>)))
  "Returns md5sum for a message object of type '<gps_msg>"
  "9a89ca61072a2258a2e6eec048f89329")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gps_msg)))
  "Returns md5sum for a message object of type 'gps_msg"
  "9a89ca61072a2258a2e6eec048f89329")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gps_msg>)))
  "Returns full string definition for message of type '<gps_msg>"
  (cl:format cl:nil "float32 latitude~%float32 longitude~%float32 speed_kmh~%float32 northing~%float32 easting~%float32 tracking_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gps_msg)))
  "Returns full string definition for message of type 'gps_msg"
  (cl:format cl:nil "float32 latitude~%float32 longitude~%float32 speed_kmh~%float32 northing~%float32 easting~%float32 tracking_angle~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gps_msg>))
  (cl:+ 0
     4
     4
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gps_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'gps_msg
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':speed_kmh (speed_kmh msg))
    (cl:cons ':northing (northing msg))
    (cl:cons ':easting (easting msg))
    (cl:cons ':tracking_angle (tracking_angle msg))
))
