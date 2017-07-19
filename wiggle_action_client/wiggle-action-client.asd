(defsystem wiggle-action-client
  :depends-on (cram-language
               cl-transforms-stamped
               actionlib
               wiggle_msgs-msg)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "wiggle-action-client" :depends-on ("package"))))))
