#! /usr/bin/env roseus

(ros::load-ros-package "roseus_resume")
(ros::roseus "roseus_resume_stop_process")

(defun generate-kill-function (pid)
  `(lambda-closure nil 0 0 (msg) (unix:kill ,pid unix::sigurg)))

(defun install-subscriber (pid topic-name message-type)
  (ros::subscribe topic-name message-type (generate-kill-function pid)))

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
(ros::spin)
