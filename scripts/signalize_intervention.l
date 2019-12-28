#! /usr/bin/env roseus

(ros::load-ros-package "roseus_resume")
(ros::roseus "roseus_resume_stop_process")

(defvar *monitoring-processes* nil
  "list of the PIDs currently monitored")
(defvar *pid-table* (make-hash-table)
  "hash-table for list of subscribed topics by PID")
(defvar check-count 0)

(defun generate-kill-function (pid)
  `(lambda-closure nil 0 0 (msg) (unix:kill ,pid unix::sigurg)))

(defun install-subscriber (pid topic-name message-type)
  (ros::subscribe topic-name message-type (generate-kill-function pid))
  (setf (gethash pid *pid-table*) (cons topic-name (gethash pid *pid-table*)))
  (pushnew pid *monitoring-processes*))

(defun check-alive ()
  (dolist (p *monitoring-processes*)
    (unless (zerop (unix::kill p 0))
      (ros::ros-warn "... Closing pid ~A" p)
      (dolist (topic (gethash p *pid-table*))
        (ros::unsubscribe topic))
      (remhash p *pid-table*)
      ;; do not use delete, which would modify the iteration list
      (setq *monitoring-processes* (remove p *monitoring-processes*)))))

(defun register-intervention-cb (msg)
  (let ((res (send msg :response)))
    (handler-case
        (let ((pid (send msg :pid))
              (topic-name (send msg :topic))
              (message-type (eval (read-from-string (send msg :type)))))
          (ros::ros-info "Registering intervention on topic ~S with PID ~S~%" topic-name pid)
          (install-subscriber pid topic-name message-type)
          (send res :success t))
      (error (err)
        (lisp::print-error-message err)
        (send res :success nil)))
    res))
      

(ros::advertise-service "/roseus_resume/register_intervention"
                        roseus_resume::RegisterIntervention
                        #'register-intervention-cb)


(ros::ros-info "Serving on /roseus_resume/register_intervention ...")
(ros::rate 100)

(while (ros::ok)
  (ros::spin-once)
  (when (> (incf check-count) 10)
    (check-alive)
    (setq check-count 0))
  (ros::sleep))
