(ros::load-ros-manifest "jsk_gui_msgs")

(unless (find-package "ROSEUS_RESUME")
  (load "package://roseus_resume/euslisp/interruption-handler.l"))
(in-package "ROSEUS_RESUME")
(export '(register-speech-intervention unregister-speech-intervention))

(defparameter *speech-intervention-alist*
  '(("stop" . #'(lambda () (signals interruption-event)))
    ("continue" . #'(lambda () (reset *replevel*)))))

(defun speech-recognition-callback (msg)
  (dolist (txt (send msg :texts))
    (let ((match (assoc txt *speech-intervention-alist* :test #'string-equal)))
      (if match (funcall (cdr match))))))

(ros::subscribe "Tablet/voice" jsk_gui_msgs::VoiceMessage
                #'speech-recognition-callback
                :groupname *intervention-groupname*)

(defmacro register-speech-intervention (word &rest body)
  `(setq *speech-intervention-alist*
         (acons word #'(lambda () ,@body) *speech-intervention-alist*)))

(defun unregister-speech-intervention (word)
  (setq *speech-intervention-alist*
        (delete word *speech-intervention-alist*
                :key #'car :test #'string-equal)))
