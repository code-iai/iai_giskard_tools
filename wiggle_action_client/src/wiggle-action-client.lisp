(in-package :wac)
(defvar *wiggle-action-client* nil)

(defparameter *wiggle-action-timeout* 20.0 "in seconds")

(defun init-wiggle-action-client ()
  (prog1
    (setf *wiggle-action-client* (actionlib:make-action-client
      "wiggle_wiggle_wiggle"
      "wiggle_msgs/WiggleAction"))
    (loop until (actionlib:wait-for-server *wiggle-action-client* 5.0)
      do (roslisp:ros-info (wiggle-action-client)
        "Waiting for wiggle actionlib server..."))
    (roslisp:ros-info (wiggle-action-client) "Wiggle action client created.")))

(defun destroy-wiggle-action-client ()
  (setf *wiggle-action-client* nil))

(defun get-wiggle-action-client ()
  (or *wiggle-action-client*
   (init-wiggle-action-client)))

(defun wiggle-feedback-cb (msg)
          (roslisp:with-fields (progress)
              msg
            (format t "~a~%" progress)))

(defun make-wiggle-action-goal (goal-pose timeout-value arm-value wiggle-type-value upperbound-x-value upperbound-y-value upperbound-angle-value cycle-time)
  (actionlib:make-action-goal
   (get-wiggle-action-client)
   GOAL_POSE (cl-transforms-stamped:to-msg goal-pose)
   TIMEOUT (roslisp:make-msg "std_msgs/Duration" (data) timeout-value)
   ARM arm-value
   WIGGLE_TYPE wiggle-type-value
   UPPERBOUND_X upperbound-x-value
   UPPERBOUND_Y upperbound-y-value
   UPPERBOUND_ANGLE upperbound-angle-value
   CYCLE_TIME cycle-time))

(defun perform-wiggle-action (goal-pose timeout-value arm-value wiggle-type-value upperbound-x-value upperbound-y-value upperbound-angle-value cycle-time)
  (actionlib:call-goal
   (get-wiggle-action-client)
   (make-wiggle-action-goal goal-pose timeout-value arm-value wiggle-type-value upperbound-x-value upperbound-y-value upperbound-angle-value cycle-time)
   :feedback-cb 'wiggle-feedback-cb ))
