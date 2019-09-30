;; Sample program for pr2 robot

;; Load pr2 interface
(load "package://pr2eus/pr2-interface.l")
(pr2-init)

;; Load interruption interface
(load "package://roseus_resume/euslisp/interruption-handler.l")
(roseus_resume:install-interruption-handler *ri*)


;; Overwrite default interruption behavior by method
;; (defmethod pr2-interface
;;   (:interrupt-angle-vector ()
;;    (send self :stop-motion)))
;; Overwrite default interruption behavior by signal
(install-handler roseus_resume:on-interruption #'(lambda (c) (send *ri* :stop-motion)))


;; Register custom intervention behaviors
(roseus_resume:register-callable-intervention
 ;; Creates a topic named `/intervention_<PID>/speak' of type `std_msgs/String'
 (:speak (msg) std_msgs::String
   (send *ri* :speak (send msg :data)))
 (:stop-speak (msg) std_msgs::String
   (let ((action-status (send *ri* :get-action-status)))
     (send *ri* :stop-motion)
     (send *ri* :speak (send msg :data) :wait t)
     (send *ri* :resume-angle-vector action-status)))
 (:stop-speak2 (msg) std_msgs::String
   (handler-bind ((interruption:on-standby
                   `(lambda-closure 0 0 nil (c) (send *ri* :speak ,(send msg :data) :wait t))))
     (signals interruption-event))))


;; Define temporary behaviors
(defun demo ()
  (handler-case
      (send *ri* :angle-vector
            (if (< (norm (v- (send *ri* :state :potentio-vector) (send *pr2* :reset-pose))) 100.0)
                (send *pr2* :init-pose)
                (send *pr2* :reset-pose))
            7000)
    (roseus_resume:on-standby () (ros::warn "Aborted demo function!"))))
