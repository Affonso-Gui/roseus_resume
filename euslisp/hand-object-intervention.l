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

(defun get-current-angle-vector-tint ()
  (if *current-status*
      (*current-status* . tint)
      user::*angle-vector-tint*))


;; Handler Function

(defvar *intersection-resume-wait-time* 0.5)

(defcondition hand-object-intersection)
(defun stop-intersection (c)
  (ros::ros-info ";; Intersection detected! Stopping...")
  (let* ((server (find-if #'(lambda (x) (send x :is-active)) roseus_bt::*action-list*))
         (goal (and server (send server :get-val "GOAL")))
         (timer (instance user::mtimer :init)))
    (when (null server)
      (ros::ros-info ";; No actions running!")
      (return-from stop-intersection nil))
    (handler-bind ((hand-object-intersection
                    #'(lambda (c) (setq timer (instance user::mtimer :init)))))
      (labels ((timer-fn ()
                 ;; only check for resumptions after a fixed wait-time has passed
                 (let ((diff (- *intersection-resume-wait-time* (send timer :stop))))
                   (if (> diff 0.0)
                       ;; create a new lambda form to avoid overlapping registration names
                       (progn
                         (ros::ros-info ";; Waiting after intersection for: ~A(s)" diff)
                         (ros::create-timer diff #'(lambda (event) (timer-fn)) :oneshot t
                                            :groupname (send user::*ri* :get-val "GROUPNAME")))
                       ;; send state to the tree and ask for a new request
                       (lisp::atomic
                           (progn
                             (ros::ros-info ";; Sending result after intersection...")
                             (send server :set-succeeded
                                   (send server :result :success nil)))
                         (condition ()))))))
        (if (> *intersection-resume-wait-time* 0)
            (progn
              ;; start timer function
              (timer-fn)
              ;; dispatch the interruption callback
              (print "Exitting...")
              (ros::ros-info ";; Stashing current progress")
              (signals roseus_bt:interrupt-action))
            ;; no wait time
            ;; canceling the action will handle everything for us
            (signals roseus_bt:cancel-action))))))
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
                            (if (assoc ,object-topic (get-current-angle-vector-tint) :test #'equal)
                                (signals hand-object-intersection)))))))

(defmacro install-hand-intersection (topic arm &optional groupname)
  ;; always stop motion, independently of manipulation state
  `(defintervention hand-object-intersection ,topic std_msgs::Empty
                    :groupname ,groupname))
