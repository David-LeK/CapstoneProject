;; Auto-generated. Do not edit!


(when (boundp 'custom_msg::encoder_input_msg)
  (if (not (find-package "CUSTOM_MSG"))
    (make-package "CUSTOM_MSG"))
  (shadow 'encoder_input_msg (find-package "CUSTOM_MSG")))
(unless (find-package "CUSTOM_MSG::ENCODER_INPUT_MSG")
  (make-package "CUSTOM_MSG::ENCODER_INPUT_MSG"))

(in-package "ROS")
;;//! \htmlinclude encoder_input_msg.msg.html


(defclass custom_msg::encoder_input_msg
  :super ros::object
  :slots (_input_setpoint_m1 _input_Kp_m1 _input_Ki_m1 _input_Kd_m1 _input_setpoint_m2 _input_Kp_m2 _input_Ki_m2 _input_Kd_m2 ))

(defmethod custom_msg::encoder_input_msg
  (:init
   (&key
    ((:input_setpoint_m1 __input_setpoint_m1) 0.0)
    ((:input_Kp_m1 __input_Kp_m1) 0.0)
    ((:input_Ki_m1 __input_Ki_m1) 0.0)
    ((:input_Kd_m1 __input_Kd_m1) 0.0)
    ((:input_setpoint_m2 __input_setpoint_m2) 0.0)
    ((:input_Kp_m2 __input_Kp_m2) 0.0)
    ((:input_Ki_m2 __input_Ki_m2) 0.0)
    ((:input_Kd_m2 __input_Kd_m2) 0.0)
    )
   (send-super :init)
   (setq _input_setpoint_m1 (float __input_setpoint_m1))
   (setq _input_Kp_m1 (float __input_Kp_m1))
   (setq _input_Ki_m1 (float __input_Ki_m1))
   (setq _input_Kd_m1 (float __input_Kd_m1))
   (setq _input_setpoint_m2 (float __input_setpoint_m2))
   (setq _input_Kp_m2 (float __input_Kp_m2))
   (setq _input_Ki_m2 (float __input_Ki_m2))
   (setq _input_Kd_m2 (float __input_Kd_m2))
   self)
  (:input_setpoint_m1
   (&optional __input_setpoint_m1)
   (if __input_setpoint_m1 (setq _input_setpoint_m1 __input_setpoint_m1)) _input_setpoint_m1)
  (:input_Kp_m1
   (&optional __input_Kp_m1)
   (if __input_Kp_m1 (setq _input_Kp_m1 __input_Kp_m1)) _input_Kp_m1)
  (:input_Ki_m1
   (&optional __input_Ki_m1)
   (if __input_Ki_m1 (setq _input_Ki_m1 __input_Ki_m1)) _input_Ki_m1)
  (:input_Kd_m1
   (&optional __input_Kd_m1)
   (if __input_Kd_m1 (setq _input_Kd_m1 __input_Kd_m1)) _input_Kd_m1)
  (:input_setpoint_m2
   (&optional __input_setpoint_m2)
   (if __input_setpoint_m2 (setq _input_setpoint_m2 __input_setpoint_m2)) _input_setpoint_m2)
  (:input_Kp_m2
   (&optional __input_Kp_m2)
   (if __input_Kp_m2 (setq _input_Kp_m2 __input_Kp_m2)) _input_Kp_m2)
  (:input_Ki_m2
   (&optional __input_Ki_m2)
   (if __input_Ki_m2 (setq _input_Ki_m2 __input_Ki_m2)) _input_Ki_m2)
  (:input_Kd_m2
   (&optional __input_Kd_m2)
   (if __input_Kd_m2 (setq _input_Kd_m2 __input_Kd_m2)) _input_Kd_m2)
  (:serialization-length
   ()
   (+
    ;; float32 _input_setpoint_m1
    4
    ;; float32 _input_Kp_m1
    4
    ;; float32 _input_Ki_m1
    4
    ;; float32 _input_Kd_m1
    4
    ;; float32 _input_setpoint_m2
    4
    ;; float32 _input_Kp_m2
    4
    ;; float32 _input_Ki_m2
    4
    ;; float32 _input_Kd_m2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _input_setpoint_m1
       (sys::poke _input_setpoint_m1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_Kp_m1
       (sys::poke _input_Kp_m1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_Ki_m1
       (sys::poke _input_Ki_m1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_Kd_m1
       (sys::poke _input_Kd_m1 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_setpoint_m2
       (sys::poke _input_setpoint_m2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_Kp_m2
       (sys::poke _input_Kp_m2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_Ki_m2
       (sys::poke _input_Ki_m2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _input_Kd_m2
       (sys::poke _input_Kd_m2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _input_setpoint_m1
     (setq _input_setpoint_m1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_Kp_m1
     (setq _input_Kp_m1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_Ki_m1
     (setq _input_Ki_m1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_Kd_m1
     (setq _input_Kd_m1 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_setpoint_m2
     (setq _input_setpoint_m2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_Kp_m2
     (setq _input_Kp_m2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_Ki_m2
     (setq _input_Ki_m2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _input_Kd_m2
     (setq _input_Kd_m2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get custom_msg::encoder_input_msg :md5sum-) "db9ba92c90b9ba885220db60c812fbcf")
(setf (get custom_msg::encoder_input_msg :datatype-) "custom_msg/encoder_input_msg")
(setf (get custom_msg::encoder_input_msg :definition-)
      "float32 input_setpoint_m1
float32 input_Kp_m1
float32 input_Ki_m1
float32 input_Kd_m1
float32 input_setpoint_m2
float32 input_Kp_m2
float32 input_Ki_m2
float32 input_Kd_m2

")



(provide :custom_msg/encoder_input_msg "db9ba92c90b9ba885220db60c812fbcf")


