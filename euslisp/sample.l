;; Sample program for pr2 robot

;; Load pr2 interface
(load "package://pr2eus/pr2-interface.l")
(pr2-init)

;; Load interruption interface
(load "package://roseus_resume/euslisp/interruption-handler.l")
;; (use-package "ROSEUS_RESUME")
;; (setq roseus_resume::*namespace* "my_namespace")
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

;; Register custom interventions

;; Creates a topic named `/roseus_resume/speak' of type `std_msgs/String'
;; and defines an intervention condition `roseus_resume::speak-message',
;; which is signalized every time a message arrives on the topic
(roseus_resume:defintervention roseus_resume::speak-message
    :message-type std_msgs::String
    :topic-name "roseus_resume/speak")

;; The ROS message is stored in the intervention condition class
;; and can be accessed through the method `:ros-msg'
;; Setting `:recursive' to nil avoid that the same handler is called again
;; if the topic is published one more time during execution
(install-handler roseus_resume::speak-message
  #'(lambda (c) (send *ri* :speak (send (send c :ros-msg) :data) :wait t))
  :recursive nil)

;; Utility to define multiple intervention callbacks
(defparameter intervention-instance
  (roseus_resume:register-callable-intervention
   ;; Stop action, speak, and then resume the action
   (:stop-speak (msg) std_msgs::String
     (let ((action-status (send *ri* :get-action-status)))
       (send *ri* :stop-motion)
       (send *ri* :speak (send msg :data) :wait t)
       (send *ri* :resume-angle-vector action-status)))
   ;; Same as `stop-speak', but incorporating the interruption logic
   (:welcome (&optional msg) std_msgs::Empty
     (handler-bind ((roseus_resume:on-standby
                     #'(lambda (c) (send *ri* :speak "Welcome to JSK" :wait t))))
       (signals interruption)))))

;; Register speech recognition interventions
(load "package://roseus_resume/euslisp/speech-recognition-intervention.l")
(roseus_resume:register-speech-intervention "hello"
  (send intervention-instance :welcome))
