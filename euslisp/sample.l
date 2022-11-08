;; Sample program for pr2 robot

;; Load pr2 interface
(load "package://pr2eus/pr2-interface.l")
(pr2-init)

;; Load interruption interface
(load "package://roseus_resume/euslisp/interruption-handler.l")
;; (use-package "ROSEUS_RESUME")
(roseus_resume:install-interruption-handler *ri*)

;; The default interruption handler will take care of all joint trajectory controllers
;; and all actionlib clients which are accessible from the robot-interface slots.
;; In order to register additional controllers, pass them as arguments during the installation
;; (roseus_resume:install-interruption-handler *ri* (gethash "robotsound" *sound-play-clients*))

#||
Installing the interruption handler overwrites sigint (C-c) behavior
to send the following three signals:

- `roseus_resume:on_interruption'
    default_callback:
      (send *ri* :interrupt-angle-vector)
      (send *ri* :interrupt-additional-controllers)
- `roseus_resume:on_standby'
    default_callback: creates a new REPL
- `roseus_resume:on_resume'
    default_callback:
      (send *ri* :resume-additional-controllers)
      (send *ri* :resume-angle-vector)
||#


;; Overwrite default interruption behavior
(install-handler roseus_resume:on-interruption #'(lambda (c) (send *ri* :stop-motion)))


;; Register interventions

#||
Interventions subscribe ROS topics on a given groupname.
Use `(*ri* . groupname)' to check at every robot spin, e.g. during wait-interpolation.
||#

;; Subscribes to a topic named `/roseus_resume/speak' of type `std_msgs/String'
;; and defines an intervention condition `roseus_resume::speak-message',
;; which is signalized every time a message arrives on the topic
(roseus_resume:defintervention roseus_resume::speak-message
    "roseus_resume/speak" std_msgs::String
    :groupname (*ri* . groupname))

;; The ROS message is stored in the intervention condition class
;; and can be accessed through the method `:ros-msg'
;; The default behavior is to recall the handler whenever a new message arrives, so
;; local handler bindings should be used to avoid recalling the handler during execution
(install-handler roseus_resume::speak-message
  #'(lambda (c)
      (handler-bind ((roseus_resume::speak-message
                      #'(lambda (c) (ros::ros-info "Ignoring: ~A" c))))
        (send *ri* :speak (send (send c :ros-msg) :data) :wait t))))


#||
Urgent interventions subscribe ROS topics on a separate thread,
raising sigint and sigcont signals whenever a message is received.
Since unix signals are checked in different points of the euslisp engine,
and are able to interrupt sleep and other system calls,
this ensures that the interruptions are delivered as soon as possible.

To setup urgent interventions, call `install-urgent-intervention-framework'.
This will also set default subscribers for the
`/roseus_resume/interrupt' and `/roseus_resume/resume' topics,
which send sigint and sigcont signals, respectively.
||#

(roseus_resume:install-urgent-intervention-framework)

;; An urgent intervention is always monitored on another thread,
;; so the signal handler is dispatched at all times, even during idle REPL
(roseus_resume:define-urgent-intervention roseus_resume::speak-message
    "roseus_resume/speak_urgent" std_msgs::String)


;; Register speech recognition interventions
(load "package://roseus_resume/euslisp/speech-recognition-intervention.l")
(roseus_resume:install-speech-intervention :groupname (*ri* . groupname))

(roseus_resume:register-speech-intervention "hello"
  (let ((action-status (send *ri* :get-action-status)))
    (send *ri* :stop-motion)
    (send *ri* :speak "Greetings, human." :wait t)
    (send *ri* :resume-angle-vector action-status)))
