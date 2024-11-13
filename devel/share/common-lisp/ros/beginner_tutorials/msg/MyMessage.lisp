; Auto-generated. Do not edit!


(cl:in-package beginner_tutorials-msg)


;//! \htmlinclude MyMessage.msg.html

(cl:defclass <MyMessage> (roslisp-msg-protocol:ros-message)
  ((time
    :reader time
    :initarg :time
    :type cl:string
    :initform "")
   (nums
    :reader nums
    :initarg :nums
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass MyMessage (<MyMessage>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyMessage>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyMessage)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name beginner_tutorials-msg:<MyMessage> is deprecated: use beginner_tutorials-msg:MyMessage instead.")))

(cl:ensure-generic-function 'time-val :lambda-list '(m))
(cl:defmethod time-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:time-val is deprecated.  Use beginner_tutorials-msg:time instead.")
  (time m))

(cl:ensure-generic-function 'nums-val :lambda-list '(m))
(cl:defmethod nums-val ((m <MyMessage>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader beginner_tutorials-msg:nums-val is deprecated.  Use beginner_tutorials-msg:nums instead.")
  (nums m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyMessage>) ostream)
  "Serializes a message object of type '<MyMessage>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'time))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'time))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'nums))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'nums))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyMessage>) istream)
  "Deserializes a message object of type '<MyMessage>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'time) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'time) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'nums) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'nums)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyMessage>)))
  "Returns string type for a message object of type '<MyMessage>"
  "beginner_tutorials/MyMessage")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyMessage)))
  "Returns string type for a message object of type 'MyMessage"
  "beginner_tutorials/MyMessage")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyMessage>)))
  "Returns md5sum for a message object of type '<MyMessage>"
  "c0c0cdfb81e8e5ff82904cd630ef5b88")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyMessage)))
  "Returns md5sum for a message object of type 'MyMessage"
  "c0c0cdfb81e8e5ff82904cd630ef5b88")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyMessage>)))
  "Returns full string definition for message of type '<MyMessage>"
  (cl:format cl:nil "string time~%int32[] nums~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyMessage)))
  "Returns full string definition for message of type 'MyMessage"
  (cl:format cl:nil "string time~%int32[] nums~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyMessage>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'time))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'nums) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyMessage>))
  "Converts a ROS message object to a list"
  (cl:list 'MyMessage
    (cl:cons ':time (time msg))
    (cl:cons ':nums (nums msg))
))
