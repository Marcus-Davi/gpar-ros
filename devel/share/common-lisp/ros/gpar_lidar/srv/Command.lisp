; Auto-generated. Do not edit!


(cl:in-package gpar_lidar-srv)


;//! \htmlinclude Command-request.msg.html

(cl:defclass <Command-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Command-request (<Command-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpar_lidar-srv:<Command-request> is deprecated: use gpar_lidar-srv:Command-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <Command-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpar_lidar-srv:command-val is deprecated.  Use gpar_lidar-srv:command instead.")
  (command m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command-request>) ostream)
  "Serializes a message object of type '<Command-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command-request>) istream)
  "Deserializes a message object of type '<Command-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command-request>)))
  "Returns string type for a service object of type '<Command-request>"
  "gpar_lidar/CommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command-request)))
  "Returns string type for a service object of type 'Command-request"
  "gpar_lidar/CommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command-request>)))
  "Returns md5sum for a message object of type '<Command-request>"
  "3a8ad692d089a2ce2722a25fc823ef5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command-request)))
  "Returns md5sum for a message object of type 'Command-request"
  "3a8ad692d089a2ce2722a25fc823ef5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command-request>)))
  "Returns full string definition for message of type '<Command-request>"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command-request)))
  "Returns full string definition for message of type 'Command-request"
  (cl:format cl:nil "int8 command~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Command-request
    (cl:cons ':command (command msg))
))
;//! \htmlinclude Command-response.msg.html

(cl:defclass <Command-response> (roslisp-msg-protocol:ros-message)
  ((command_res
    :reader command_res
    :initarg :command_res
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Command-response (<Command-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Command-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Command-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name gpar_lidar-srv:<Command-response> is deprecated: use gpar_lidar-srv:Command-response instead.")))

(cl:ensure-generic-function 'command_res-val :lambda-list '(m))
(cl:defmethod command_res-val ((m <Command-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader gpar_lidar-srv:command_res-val is deprecated.  Use gpar_lidar-srv:command_res instead.")
  (command_res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Command-response>) ostream)
  "Serializes a message object of type '<Command-response>"
  (cl:let* ((signed (cl:slot-value msg 'command_res)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Command-response>) istream)
  "Deserializes a message object of type '<Command-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command_res) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Command-response>)))
  "Returns string type for a service object of type '<Command-response>"
  "gpar_lidar/CommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command-response)))
  "Returns string type for a service object of type 'Command-response"
  "gpar_lidar/CommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Command-response>)))
  "Returns md5sum for a message object of type '<Command-response>"
  "3a8ad692d089a2ce2722a25fc823ef5f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Command-response)))
  "Returns md5sum for a message object of type 'Command-response"
  "3a8ad692d089a2ce2722a25fc823ef5f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Command-response>)))
  "Returns full string definition for message of type '<Command-response>"
  (cl:format cl:nil "int8 command_res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Command-response)))
  "Returns full string definition for message of type 'Command-response"
  (cl:format cl:nil "int8 command_res~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Command-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Command-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Command-response
    (cl:cons ':command_res (command_res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Command)))
  'Command-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Command)))
  'Command-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Command)))
  "Returns string type for a service object of type '<Command>"
  "gpar_lidar/Command")