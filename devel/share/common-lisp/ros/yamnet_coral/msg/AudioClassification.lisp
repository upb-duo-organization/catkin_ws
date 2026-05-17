; Auto-generated. Do not edit!


(cl:in-package yamnet_coral-msg)


;//! \htmlinclude AudioClassification.msg.html

(cl:defclass <AudioClassification> (roslisp-msg-protocol:ros-message)
  ((label
    :reader label
    :initarg :label
    :type cl:string
    :initform "")
   (confidence
    :reader confidence
    :initarg :confidence
    :type cl:float
    :initform 0.0)
   (rms
    :reader rms
    :initarg :rms
    :type cl:float
    :initform 0.0)
   (stamp
    :reader stamp
    :initarg :stamp
    :type cl:real
    :initform 0))
)

(cl:defclass AudioClassification (<AudioClassification>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AudioClassification>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AudioClassification)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name yamnet_coral-msg:<AudioClassification> is deprecated: use yamnet_coral-msg:AudioClassification instead.")))

(cl:ensure-generic-function 'label-val :lambda-list '(m))
(cl:defmethod label-val ((m <AudioClassification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yamnet_coral-msg:label-val is deprecated.  Use yamnet_coral-msg:label instead.")
  (label m))

(cl:ensure-generic-function 'confidence-val :lambda-list '(m))
(cl:defmethod confidence-val ((m <AudioClassification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yamnet_coral-msg:confidence-val is deprecated.  Use yamnet_coral-msg:confidence instead.")
  (confidence m))

(cl:ensure-generic-function 'rms-val :lambda-list '(m))
(cl:defmethod rms-val ((m <AudioClassification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yamnet_coral-msg:rms-val is deprecated.  Use yamnet_coral-msg:rms instead.")
  (rms m))

(cl:ensure-generic-function 'stamp-val :lambda-list '(m))
(cl:defmethod stamp-val ((m <AudioClassification>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader yamnet_coral-msg:stamp-val is deprecated.  Use yamnet_coral-msg:stamp instead.")
  (stamp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AudioClassification>) ostream)
  "Serializes a message object of type '<AudioClassification>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'label))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'label))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'rms))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__sec (cl:floor (cl:slot-value msg 'stamp)))
        (__nsec (cl:round (cl:* 1e9 (cl:- (cl:slot-value msg 'stamp) (cl:floor (cl:slot-value msg 'stamp)))))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __sec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 0) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __nsec) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __nsec) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AudioClassification>) istream)
  "Deserializes a message object of type '<AudioClassification>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'label) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'label) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'confidence) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rms) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((__sec 0) (__nsec 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __sec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 0) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __nsec) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __nsec) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stamp) (cl:+ (cl:coerce __sec 'cl:double-float) (cl:/ __nsec 1e9))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AudioClassification>)))
  "Returns string type for a message object of type '<AudioClassification>"
  "yamnet_coral/AudioClassification")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AudioClassification)))
  "Returns string type for a message object of type 'AudioClassification"
  "yamnet_coral/AudioClassification")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AudioClassification>)))
  "Returns md5sum for a message object of type '<AudioClassification>"
  "10f8093929cee2e0e62a2607a78d22df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AudioClassification)))
  "Returns md5sum for a message object of type 'AudioClassification"
  "10f8093929cee2e0e62a2607a78d22df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AudioClassification>)))
  "Returns full string definition for message of type '<AudioClassification>"
  (cl:format cl:nil "string label~%float32 confidence~%float32 rms~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AudioClassification)))
  "Returns full string definition for message of type 'AudioClassification"
  (cl:format cl:nil "string label~%float32 confidence~%float32 rms~%time stamp~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AudioClassification>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'label))
     4
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AudioClassification>))
  "Converts a ROS message object to a list"
  (cl:list 'AudioClassification
    (cl:cons ':label (label msg))
    (cl:cons ':confidence (confidence msg))
    (cl:cons ':rms (rms msg))
    (cl:cons ':stamp (stamp msg))
))
