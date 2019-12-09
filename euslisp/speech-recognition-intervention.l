(ros::load-ros-manifest "jsk_gui_msgs")

(unless (find-package "ROSEUS_RESUME")
  (load "package://roseus_resume/euslisp/interruption-handler.l"))
(in-package "ROSEUS_RESUME")
(export '(register-speech-intervention unregister-speech-intervention))

(defparameter *speech-intervention-alist*
  '(("stop" . (lambda-closure nil 0 0 nil (signals interruption-event)))
    ("continue" . (lambda-closure nil 0 0 nil (reset *replevel*)))))

(defun speech-recognition-callback (texts)
  (dolist (txt texts)
    (let ((match (assoc txt *speech-intervention-alist* :test #'string-equal)))
      (if match (funcall (cdr match))))))

(defintervention speech-recognition-intervention 
    :topic-name "Tablet/voice"
    :message-type jsk_gui_msgs::VoiceMessage)
(install-handler speech-recognition-intervention 
  '(lambda-closure nil 0 0 (c) (speech-recognition-callback (send (send c :ros-msg) :texts))))

(defmacro register-speech-intervention (word &rest body)
  `(setq *speech-intervention-alist*
         (acons ,word #'(lambda () ,@body) *speech-intervention-alist*)))

(defun unregister-speech-intervention (word)
  (setq *speech-intervention-alist*
        (delete word *speech-intervention-alist*
                :key #'car :test #'string-equal)))
