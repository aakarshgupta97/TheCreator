; Auto-generated. Do not edit!


(cl:in-package planning-srv)


;//! \htmlinclude ModelGenerator-request.msg.html

(cl:defclass <ModelGenerator-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ModelGenerator-request (<ModelGenerator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelGenerator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelGenerator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-srv:<ModelGenerator-request> is deprecated: use planning-srv:ModelGenerator-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelGenerator-request>) ostream)
  "Serializes a message object of type '<ModelGenerator-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelGenerator-request>) istream)
  "Deserializes a message object of type '<ModelGenerator-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelGenerator-request>)))
  "Returns string type for a service object of type '<ModelGenerator-request>"
  "planning/ModelGeneratorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelGenerator-request)))
  "Returns string type for a service object of type 'ModelGenerator-request"
  "planning/ModelGeneratorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelGenerator-request>)))
  "Returns md5sum for a message object of type '<ModelGenerator-request>"
  "f50b0934fbc5c3c4cf0e84c00604ffe4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelGenerator-request)))
  "Returns md5sum for a message object of type 'ModelGenerator-request"
  "f50b0934fbc5c3c4cf0e84c00604ffe4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelGenerator-request>)))
  "Returns full string definition for message of type '<ModelGenerator-request>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelGenerator-request)))
  "Returns full string definition for message of type 'ModelGenerator-request"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelGenerator-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelGenerator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelGenerator-request
))
;//! \htmlinclude ModelGenerator-response.msg.html

(cl:defclass <ModelGenerator-response> (roslisp-msg-protocol:ros-message)
  ((errorCode
    :reader errorCode
    :initarg :errorCode
    :type cl:fixnum
    :initform 0)
   (blocks
    :reader blocks
    :initarg :blocks
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0))
   (num_layers
    :reader num_layers
    :initarg :num_layers
    :type cl:fixnum
    :initform 0)
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

(cl:defclass ModelGenerator-response (<ModelGenerator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ModelGenerator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ModelGenerator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planning-srv:<ModelGenerator-response> is deprecated: use planning-srv:ModelGenerator-response instead.")))

(cl:ensure-generic-function 'errorCode-val :lambda-list '(m))
(cl:defmethod errorCode-val ((m <ModelGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:errorCode-val is deprecated.  Use planning-srv:errorCode instead.")
  (errorCode m))

(cl:ensure-generic-function 'blocks-val :lambda-list '(m))
(cl:defmethod blocks-val ((m <ModelGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:blocks-val is deprecated.  Use planning-srv:blocks instead.")
  (blocks m))

(cl:ensure-generic-function 'num_layers-val :lambda-list '(m))
(cl:defmethod num_layers-val ((m <ModelGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:num_layers-val is deprecated.  Use planning-srv:num_layers instead.")
  (num_layers m))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ModelGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:width-val is deprecated.  Use planning-srv:width instead.")
  (width m))

(cl:ensure-generic-function 'height-val :lambda-list '(m))
(cl:defmethod height-val ((m <ModelGenerator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planning-srv:height-val is deprecated.  Use planning-srv:height instead.")
  (height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ModelGenerator-response>) ostream)
  "Serializes a message object of type '<ModelGenerator-response>"
  (cl:let* ((signed (cl:slot-value msg 'errorCode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'blocks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'blocks))
  (cl:let* ((signed (cl:slot-value msg 'num_layers)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ModelGenerator-response>) istream)
  "Deserializes a message object of type '<ModelGenerator-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'errorCode) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
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
      (cl:setf (cl:slot-value msg 'num_layers) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'width) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'height) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ModelGenerator-response>)))
  "Returns string type for a service object of type '<ModelGenerator-response>"
  "planning/ModelGeneratorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelGenerator-response)))
  "Returns string type for a service object of type 'ModelGenerator-response"
  "planning/ModelGeneratorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ModelGenerator-response>)))
  "Returns md5sum for a message object of type '<ModelGenerator-response>"
  "f50b0934fbc5c3c4cf0e84c00604ffe4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ModelGenerator-response)))
  "Returns md5sum for a message object of type 'ModelGenerator-response"
  "f50b0934fbc5c3c4cf0e84c00604ffe4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ModelGenerator-response>)))
  "Returns full string definition for message of type '<ModelGenerator-response>"
  (cl:format cl:nil "int8 errorCode~%int8[] blocks~%int8 num_layers~%int8 width~%int8 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ModelGenerator-response)))
  "Returns full string definition for message of type 'ModelGenerator-response"
  (cl:format cl:nil "int8 errorCode~%int8[] blocks~%int8 num_layers~%int8 width~%int8 height~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ModelGenerator-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'blocks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ModelGenerator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ModelGenerator-response
    (cl:cons ':errorCode (errorCode msg))
    (cl:cons ':blocks (blocks msg))
    (cl:cons ':num_layers (num_layers msg))
    (cl:cons ':width (width msg))
    (cl:cons ':height (height msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ModelGenerator)))
  'ModelGenerator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ModelGenerator)))
  'ModelGenerator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ModelGenerator)))
  "Returns string type for a service object of type '<ModelGenerator>"
  "planning/ModelGenerator")