;; Sample program for pr2 robot

;; Load pr2 interface
(load "package://pr2eus/pr2-interface.l")
(pr2-init)

;; Load interruption interface
(load "package://roseus_resume/euslisp/interruption-handler.l")
;; (setq roseus_resume:*namespace* "intervention")
(roseus_resume:install-interruption-handler *ri*)

#||
Installing the interruption handler overwrites sigint (C-c) behavior
to send the following three signals:

- `roseus_resume:on_interruption'
    default_callback: (send *ri* :interrupt-angle-vector)
- `roseus_resume:on_standby'
    default_callback: creates a new REPL
- `roseus_resume:on_resume'
    default_callback: (send *ri* :resume-angle-vector)

It also subscribes the ROS topics `/roseus_resume/interrupt' and `/roseus_resume/resume',
which send sigint and sigcont to the EusLisp process.
||#


;; Overwrite default interruption behavior
(install-handler roseus_resume:on-interruption #'(lambda (c) (send *ri* :stop-motion)))

;; Locally overwrite interruption behaviors
(defun demo ()
  (handler-case
      (prog1
        (send *ri* :angle-vector (send *pr2* :reset-pose) 7000)
        (send *ri* :wait-interpolation))
    (roseus_resume:on-standby () (ros::warn "Aborted demo function!"))))


;; Register custom interventions
(defparameter intervention-instance
  (roseus_resume:register-callable-intervention
   ;; Creates a topic named `/roseus_resume/speak' of type `std_msgs/String',
   ;; which signalizes an intervention condition `roseus_resume::speak' when published
   ;; and triggers the handler function (send *ri* :speak (send msg :data))
   (:speak (msg) std_msgs::String
     (send *ri* :speak (send msg :data)))
   (:stop-speak (msg) std_msgs::String
     (let ((action-status (send *ri* :get-action-status)))
       (send *ri* :stop-motion)
       (send *ri* :speak (send msg :data) :wait t)
       (send *ri* :resume-angle-vector action-status)))
   (:welcome (&optional msg) std_msgs::Empty
     (handler-bind ((roseus_resume:on-standby
                     #'(lambda (c) (send *ri* :speak "Welcome to JSK" :wait t))))
       (signals interruption)))))

;; Register speech recognition interventions
(load "package://roseus_resume/euslisp/speech-recognition-intervention.l")
(roseus_resume:register-speech-intervention "hello"
  (send intervention-instance :welcome))
