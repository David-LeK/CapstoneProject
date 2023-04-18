
(cl:in-package :asdf)

(defsystem "custom_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "encoder_input_msg" :depends-on ("_package_encoder_input_msg"))
    (:file "_package_encoder_input_msg" :depends-on ("_package"))
    (:file "encoder_output_msg" :depends-on ("_package_encoder_output_msg"))
    (:file "_package_encoder_output_msg" :depends-on ("_package"))
    (:file "mpu_msg" :depends-on ("_package_mpu_msg"))
    (:file "_package_mpu_msg" :depends-on ("_package"))
  ))