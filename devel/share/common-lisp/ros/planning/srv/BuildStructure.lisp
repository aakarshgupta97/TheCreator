; Auto-generated. Do not edit!


(cl:in-package planning-srv)


;//! \htmlinclude BuildStructure-request.msg.html

(cl:defclass <BuildStructure-request> (roslisp-msg-protocol:ros-message)
  ((blocks
    :reader blocks
    :initarg :blocks
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (width
    :reader width
    :initarg :width
    :type cl:fixnum
    :initform 0)
   (height
    :reader height
    :initarg :height
    :type cl:fixnum
    :initform 0))
)

(cl:defclass BuildStructure-request (<BuildStructure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BuildStructure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BuildStructure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-srv:<BuildStructure-request> is deprecated: use planning-srv:BuildStructure-request instead.")))

(cl:ensure-generic-function 'blocks-val :lambda-list '(m))
(cl:defmethod blocks-val ((m <BuildStructure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:blocks-val is deprecated.  Use planning-srv:blocks instead.")
  (blocks m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <BuildStructure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:width-val is deprecated.  Use planning-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <BuildStructure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:height-val is deprecated.  Use planning-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BuildStructure-request>) ostream)
  "Serializes a message object of type '<BuildStructure-request>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blocks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'blocks))
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BuildStructure-request>) istream)
  "Deserializes a message object of type '<BuildStructure-request>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'blocks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'blocks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BuildStructure-request>)))
  "Returns string type for a service object of type '<BuildStructure-request>"
  "planning/BuildStructureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BuildStructure-request)))
  "Returns string type for a service object of type 'BuildStructure-request"
  "planning/BuildStructureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BuildStructure-request>)))
  "Returns md5sum for a message object of type '<BuildStructure-request>"
  "94a3098ba73f0d4168f95004e9a559df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BuildStructure-request)))
  "Returns md5sum for a message object of type 'BuildStructure-request"
  "94a3098ba73f0d4168f95004e9a559df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BuildStructure-request>)))
  "Returns full string definition for message of type '<BuildStructure-request>"
  (cl:format cl:nil "int8[] blocks~%int8 width~%int8 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BuildStructure-request)))
  "Returns full string definition for message of type 'BuildStructure-request"
  (cl:format cl:nil "int8[] blocks~%int8 width~%int8 height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BuildStructure-request>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blocks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BuildStructure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'BuildStructure-request
    (cl:cons ':blocks (blocks msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
;//! \htmlinclude BuildStructure-response.msg.html

(cl:defclass <BuildStructure-response> (roslisp-msg-protocol:ros-message)
  ((errorCode
    :reader errorCode
    :initarg :errorCode
    :type cl:fixnum
    :initform 0))
)

(cl:defclass BuildStructure-response (<BuildStructure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BuildStructure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BuildStructure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-srv:<BuildStructure-response> is deprecated: use planning-srv:BuildStructure-response instead.")))

(cl:ensure-generic-function 'errorCode-val :lambda-list '(m))
(cl:defmethod errorCode-val ((m <BuildStructure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:errorCode-val is deprecated.  Use planning-srv:errorCode instead.")
  (errorCode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BuildStructure-response>) ostream)
  "Serializes a message object of type '<BuildStructure-response>"
  (cl:let* ((signed (cl:slot-value msg 'errorCode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BuildStructure-response>) istream)
  "Deserializes a message object of type '<BuildStructure-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errorCode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BuildStructure-response>)))
  "Returns string type for a service object of type '<BuildStructure-response>"
  "planning/BuildStructureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BuildStructure-response)))
  "Returns string type for a service object of type 'BuildStructure-response"
  "planning/BuildStructureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BuildStructure-response>)))
  "Returns md5sum for a message object of type '<BuildStructure-response>"
  "94a3098ba73f0d4168f95004e9a559df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BuildStructure-response)))
  "Returns md5sum for a message object of type 'BuildStructure-response"
  "94a3098ba73f0d4168f95004e9a559df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BuildStructure-response>)))
  "Returns full string definition for message of type '<BuildStructure-response>"
  (cl:format cl:nil "int8 errorCode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BuildStructure-response)))
  "Returns full string definition for message of type 'BuildStructure-response"
  (cl:format cl:nil "int8 errorCode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BuildStructure-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BuildStructure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'BuildStructure-response
    (cl:cons ':errorCode (errorCode msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'BuildStructure)))
  'BuildStructure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'BuildStructure)))
  'BuildStructure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BuildStructure)))
  "Returns string type for a service object of type '<BuildStructure>"
  "planning/BuildStructure")