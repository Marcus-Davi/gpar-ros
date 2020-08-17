;; Auto-generated. Do not edit!


(when (boundp 'gpar_lidar::Command)
  (if (not (find-package "GPAR_LIDAR"))
    (make-package "GPAR_LIDAR"))
  (shadow 'Command (find-package "GPAR_LIDAR")))
(unless (find-package "GPAR_LIDAR::COMMAND")
  (make-package "GPAR_LIDAR::COMMAND"))
(unless (find-package "GPAR_LIDAR::COMMANDREQUEST")
  (make-package "GPAR_LIDAR::COMMANDREQUEST"))
(unless (find-package "GPAR_LIDAR::COMMANDRESPONSE")
  (make-package "GPAR_LIDAR::COMMANDRESPONSE"))

(in-package "ROS")





(defclass gpar_lidar::CommandRequest
  :super ros::object
  :slots (_command ))

(defmethod gpar_lidar::CommandRequest
  (:init
   (&key
    ((:command __command) 0)
    )
   (send-super :init)
   (setq _command (round __command))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:serialization-length
   ()
   (+
    ;; int8 _command
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _command
       (write-byte _command s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _command
     (setq _command (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _command 127) (setq _command (- _command 256)))
   ;;
   self)
  )

(defclass gpar_lidar::CommandResponse
  :super ros::object
  :slots (_command_res ))

(defmethod gpar_lidar::CommandResponse
  (:init
   (&key
    ((:command_res __command_res) 0)
    )
   (send-super :init)
   (setq _command_res (round __command_res))
   self)
  (:command_res
   (&optional __command_res)
   (if __command_res (setq _command_res __command_res)) _command_res)
  (:serialization-length
   ()
   (+
    ;; int8 _command_res
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int8 _command_res
       (write-byte _command_res s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int8 _command_res
     (setq _command_res (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _command_res 127) (setq _command_res (- _command_res 256)))
   ;;
   self)
  )

(defclass gpar_lidar::Command
  :super ros::object
  :slots ())

(setf (get gpar_lidar::Command :md5sum-) "3a8ad692d089a2ce2722a25fc823ef5f")
(setf (get gpar_lidar::Command :datatype-) "gpar_lidar/Command")
(setf (get gpar_lidar::Command :request) gpar_lidar::CommandRequest)
(setf (get gpar_lidar::Command :response) gpar_lidar::CommandResponse)

(defmethod gpar_lidar::CommandRequest
  (:response () (instance gpar_lidar::CommandResponse :init)))

(setf (get gpar_lidar::CommandRequest :md5sum-) "3a8ad692d089a2ce2722a25fc823ef5f")
(setf (get gpar_lidar::CommandRequest :datatype-) "gpar_lidar/CommandRequest")
(setf (get gpar_lidar::CommandRequest :definition-)
      "int8 command
---
int8 command_res

")

(setf (get gpar_lidar::CommandResponse :md5sum-) "3a8ad692d089a2ce2722a25fc823ef5f")
(setf (get gpar_lidar::CommandResponse :datatype-) "gpar_lidar/CommandResponse")
(setf (get gpar_lidar::CommandResponse :definition-)
      "int8 command
---
int8 command_res

")



(provide :gpar_lidar/Command "3a8ad692d089a2ce2722a25fc823ef5f")


