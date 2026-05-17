;; Auto-generated. Do not edit!


(when (boundp 'yamnet_coral::AudioClassification)
  (if (not (find-package "YAMNET_CORAL"))
    (make-package "YAMNET_CORAL"))
  (shadow 'AudioClassification (find-package "YAMNET_CORAL")))
(unless (find-package "YAMNET_CORAL::AUDIOCLASSIFICATION")
  (make-package "YAMNET_CORAL::AUDIOCLASSIFICATION"))

(in-package "ROS")
;;//! \htmlinclude AudioClassification.msg.html


(defclass yamnet_coral::AudioClassification
  :super ros::object
  :slots (_label _confidence _rms _stamp ))

(defmethod yamnet_coral::AudioClassification
  (:init
   (&key
    ((:label __label) "")
    ((:confidence __confidence) 0.0)
    ((:rms __rms) 0.0)
    ((:stamp __stamp) (instance ros::time :init))
    )
   (send-super :init)
   (setq _label (string __label))
   (setq _confidence (float __confidence))
   (setq _rms (float __rms))
   (setq _stamp __stamp)
   self)
  (:label
   (&optional __label)
   (if __label (setq _label __label)) _label)
  (:confidence
   (&optional __confidence)
   (if __confidence (setq _confidence __confidence)) _confidence)
  (:rms
   (&optional __rms)
   (if __rms (setq _rms __rms)) _rms)
  (:stamp
   (&optional __stamp)
   (if __stamp (setq _stamp __stamp)) _stamp)
  (:serialization-length
   ()
   (+
    ;; string _label
    4 (length _label)
    ;; float32 _confidence
    4
    ;; float32 _rms
    4
    ;; time _stamp
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _label
       (write-long (length _label) s) (princ _label s)
     ;; float32 _confidence
       (sys::poke _confidence (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _rms
       (sys::poke _rms (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; time _stamp
       (write-long (send _stamp :sec) s) (write-long (send _stamp :nsec) s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _label
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _label (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _confidence
     (setq _confidence (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _rms
     (setq _rms (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; time _stamp
     (send _stamp :sec (sys::peek buf ptr- :integer)) (incf ptr- 4)  (send _stamp :nsec (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yamnet_coral::AudioClassification :md5sum-) "10f8093929cee2e0e62a2607a78d22df")
(setf (get yamnet_coral::AudioClassification :datatype-) "yamnet_coral/AudioClassification")
(setf (get yamnet_coral::AudioClassification :definition-)
      "string label
float32 confidence
float32 rms
time stamp

")



(provide :yamnet_coral/AudioClassification "10f8093929cee2e0e62a2607a78d22df")


