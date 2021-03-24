
(cl:in-package :asdf)

(defsystem "formal_control-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "PathMsg" :depends-on ("_package_PathMsg"))
    (:file "_package_PathMsg" :depends-on ("_package"))
    (:file "PathMsg" :depends-on ("_package_PathMsg"))
    (:file "_package_PathMsg" :depends-on ("_package"))
    (:file "SelfStateMsg" :depends-on ("_package_SelfStateMsg"))
    (:file "_package_SelfStateMsg" :depends-on ("_package"))
    (:file "SelfStateMsg" :depends-on ("_package_SelfStateMsg"))
    (:file "_package_SelfStateMsg" :depends-on ("_package"))
  ))