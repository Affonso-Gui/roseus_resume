(unless (find-package "ROSEUS_RESUME")
  (load "package://roseus_resume/euslisp/interruption-handler.l"))

(unless (find-package "ROSEUS_BT")
  (load "package://roseus_bt/euslisp/nodes.l"))

(in-package "ROSEUS_RESUME")
(export '(*intersection-resume-wait-time* hand-object-intersection
          install-object-intersection install-hand-intersection))

(when (not (fboundp 'ros::unresolve-name))
  (defun ros::unresolve-name (name)
    (substitute #\- #\_ (substitute #\- #\/ (string-trim "/" name)))))


;; Handler Function

(defvar *intersection-resume-wait-time* 0.5)

(defcondition hand-object-intersection)
(defun stop-intersection (c)
  (let* ((server (find-if #'(lambda (x) (send x :is-active)) roseus_bt::*action-list*))
         (goal (send server :get-val "GOAL"))
         (timer (instance user::mtimer :init)))
    (handler-bind ((hand-intersection
                    #'(lambda (c) (setq timer (instance user::mtimer :init)))))
      ;; dispatch the interruption callback
      (signals roseus_bt:interrupt-action)
      ;; only check for resumptions after a fixed wait-time has passed
      (until (> (send timer :stop) *resume-intersection-wait-time*)
             (send *ri* :spin-once)
             (roseus_bt:spin-once)
             (ros::sleep))
      (lisp::atomic
          (send server :set-succeeded
                (send server :result :success nil))
        (condition ())))))
(install-handler hand-object-intersection 'stop-intersection)


;; Install Macros

(defmacro install-object-intersection (topic object-topic &optional groupname)
  (let ((condition-name
         (intern (string-upcase (ros::unresolve-name topic)) (find-package "ROSEUS_RESUME"))))
    `(progn
       (defintervention ,condition-name ,topic std_msgs::Empty
                        :groupname ,groupname)
       (install-handler ,condition-name
                        #'(lambda (c)
                            (if (assoc ,object-topic user::*angle-vector-tint* :test #'equal)
                                (signals hand-object-intersection)))))))

(defmacro install-hand-intersection (topic arm &optional groupname)
  ;; always stop motion, independently of manipulation state
  `(defintervention hand-object-intersection ,topic std_msgs::Empty
                    :groupname ,groupname))
