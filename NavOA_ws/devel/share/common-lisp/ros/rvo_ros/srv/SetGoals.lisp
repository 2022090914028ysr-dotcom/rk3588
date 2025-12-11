; Auto-generated. Do not edit!


(cl:in-package rvo_ros-srv)


;//! \htmlinclude SetGoals-request.msg.html

(cl:defclass <SetGoals-request> (roslisp-msg-protocol:ros-message)
  ((model
    :reader model
    :initarg :model
    :type cl:string
    :initform "")
   (coordinates
    :reader coordinates
    :initarg :coordinates
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass SetGoals-request (<SetGoals-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGoals-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGoals-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvo_ros-srv:<SetGoals-request> is deprecated: use rvo_ros-srv:SetGoals-request instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <SetGoals-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvo_ros-srv:model-val is deprecated.  Use rvo_ros-srv:model instead.")
  (model m))

(cl:ensure-generic-function 'coordinates-val :lambda-list '(m))
(cl:defmethod coordinates-val ((m <SetGoals-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvo_ros-srv:coordinates-val is deprecated.  Use rvo_ros-srv:coordinates instead.")
  (coordinates m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGoals-request>) ostream)
  "Serializes a message object of type '<SetGoals-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'model))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'model))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'coordinates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'coordinates))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGoals-request>) istream)
  "Deserializes a message object of type '<SetGoals-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'model) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'coordinates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'coordinates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGoals-request>)))
  "Returns string type for a service object of type '<SetGoals-request>"
  "rvo_ros/SetGoalsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGoals-request)))
  "Returns string type for a service object of type 'SetGoals-request"
  "rvo_ros/SetGoalsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGoals-request>)))
  "Returns md5sum for a message object of type '<SetGoals-request>"
  "269a0b03d7a509c517f6ff76c318c9eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGoals-request)))
  "Returns md5sum for a message object of type 'SetGoals-request"
  "269a0b03d7a509c517f6ff76c318c9eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGoals-request>)))
  "Returns full string definition for message of type '<SetGoals-request>"
  (cl:format cl:nil "string model~%geometry_msgs/Point[] coordinates~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGoals-request)))
  "Returns full string definition for message of type 'SetGoals-request"
  (cl:format cl:nil "string model~%geometry_msgs/Point[] coordinates~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGoals-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'model))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'coordinates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGoals-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGoals-request
    (cl:cons ':model (model msg))
    (cl:cons ':coordinates (coordinates msg))
))
;//! \htmlinclude SetGoals-response.msg.html

(cl:defclass <SetGoals-response> (roslisp-msg-protocol:ros-message)
  ((num_goal
    :reader num_goal
    :initarg :num_goal
    :type cl:integer
    :initform 0))
)

(cl:defclass SetGoals-response (<SetGoals-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetGoals-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetGoals-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rvo_ros-srv:<SetGoals-response> is deprecated: use rvo_ros-srv:SetGoals-response instead.")))

(cl:ensure-generic-function 'num_goal-val :lambda-list '(m))
(cl:defmethod num_goal-val ((m <SetGoals-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rvo_ros-srv:num_goal-val is deprecated.  Use rvo_ros-srv:num_goal instead.")
  (num_goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetGoals-response>) ostream)
  "Serializes a message object of type '<SetGoals-response>"
  (cl:let* ((signed (cl:slot-value msg 'num_goal)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetGoals-response>) istream)
  "Deserializes a message object of type '<SetGoals-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_goal) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetGoals-response>)))
  "Returns string type for a service object of type '<SetGoals-response>"
  "rvo_ros/SetGoalsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGoals-response)))
  "Returns string type for a service object of type 'SetGoals-response"
  "rvo_ros/SetGoalsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetGoals-response>)))
  "Returns md5sum for a message object of type '<SetGoals-response>"
  "269a0b03d7a509c517f6ff76c318c9eb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetGoals-response)))
  "Returns md5sum for a message object of type 'SetGoals-response"
  "269a0b03d7a509c517f6ff76c318c9eb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetGoals-response>)))
  "Returns full string definition for message of type '<SetGoals-response>"
  (cl:format cl:nil "int64 num_goal~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetGoals-response)))
  "Returns full string definition for message of type 'SetGoals-response"
  (cl:format cl:nil "int64 num_goal~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetGoals-response>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetGoals-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetGoals-response
    (cl:cons ':num_goal (num_goal msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetGoals)))
  'SetGoals-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetGoals)))
  'SetGoals-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetGoals)))
  "Returns string type for a service object of type '<SetGoals>"
  "rvo_ros/SetGoals")