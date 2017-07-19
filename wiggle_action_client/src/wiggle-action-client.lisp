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

(defun make-wiggle-action-goal (goal-pose)
  (cl-transforms-stamped:to-msg goal-pose))
