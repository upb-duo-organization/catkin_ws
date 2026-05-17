
(cl:in-package :asdf)

(defsystem "yamnet_coral-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AudioClassification" :depends-on ("_package_AudioClassification"))
    (:file "_package_AudioClassification" :depends-on ("_package"))
  ))