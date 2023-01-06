(unless (find-package "ROSEUS_RESUME")
  (load "package://roseus_resume/euslisp/interruption-handler.l"))

(in-package "USER")

(defun reset-lstm-models-from-tint (ckpt)
  ;; reset reaching models
  (let ((limbs (mapcar #'car (remove-if-not #'keywordp (ckpt . tint) :key #'car)))
        (obj (car (get-tint-mark (ckpt . tint)))))
    (dolist (limb limbs)
      (if (and obj
               (memq (send ckpt :motion-intent obj) '(:absolute :relative)))
          (roseus_resume::reset-lstm-models limb obj))))
  ;; reset manipulation models
  (dolist (mark (ckpt . gripper-tint))
    (let ((limb (car mark))
          (obj (cadr mark)))
      (if (and limb obj
               (memq (send ckpt :motion-intent obj) '(:absolute :relative)))
          (roseus_resume::reset-lstm-models limb obj)))))

(push 'reset-lstm-models-from-tint *motion-checkpoint-init-callback-list*)


(in-package "ROSEUS_RESUME")
(export '(install-lstm-model reset-lstm-models reset-all-lstm-models))

(defvar *reset-lstm-service-list* nil)
;; ((limb topic) . service-name)

(when (not (fboundp 'ros::unresolve-name))
  (defun ros::unresolve-name (name)
    (substitute #\- #\_ (substitute #\- #\/ (string-trim "/" name)))))

(defcondition lstm-intervention :super intervention)

;; Reset Functions

(defun reset-lstm-models (arm object)
  (handler-bind ((lstm-intervention #'identity))
    (dolist (srv *reset-lstm-service-list*)
      (multiple-value-bind (limb topic) (car srv)
        (when (and (equal limb arm) (equal topic object))
          (ros::ros-info ";; Resetting lstm model ~A" (cdr srv))
          (ros::service-call (cdr srv) (instance std_srvs::TriggerRequest :init)))))))

(defun reset-all-lstm-models ()
  (handler-bind ((lstm-intervention #'identity))
    (dolist (srv *reset-lstm-service-list*)
      (ros::ros-info ";; Resetting lstm model ~A" (cdr srv))
      (ros::service-call (cdr srv) (instance std_srvs::TriggerRequest :init)))))


;; Install Macros

(defmacro install-lstm-intervention (topic condition-name predicate
                                     &optional groupname &rest init-args)
  (let ((lstm-condition-name
         (intern (string-upcase (ros::unresolve-name topic)) (find-package "ROSEUS_RESUME"))))
    `(progn
       (unless (boundp ',lstm-condition-name)
         (defcondition ,lstm-condition-name :super lstm-intervention))
       (defintervention ,lstm-condition-name ,topic std_msgs::Empty
                        :groupname ,groupname)
       (install-handler ,lstm-condition-name
          #'(lambda (c)
              (handler-bind ((,lstm-intervention #'identity))
                (if ,predicate
                    (signals ,condition-name ,@init-args))))))))

(defmacro install-lstm-manipulation-intervention (model-topic arm object-topic condition-name
                                                  &optional groupname &rest init-args)
  `(install-lstm-intervention ,model-topic ,condition-name
     (equal (cadr (assoc ,arm user::*gripper-tint*)) ,object-topic)
     ,groupname ,@init-args))

(defmacro install-lstm-nocontact-intervention (model-topic arm object-topic condition-name
                                               &optional groupname &rest init-args)
  `(install-lstm-intervention ,model-topic ,condition-name
     (and (assoc ,arm user::*angle-vector-tint*)
          (assoc ,object-topic user::*angle-vector-tint* :test #'equal))
     ,groupname ,@init-args))

(defmacro install-lstm-model (model-namespace arm object-topic condition-name
                              &optional groupname &rest init-args)
  ;; https://github.com/Affonso-Gui/epic-project/blob/master/src/lstm_ros_inference.py
  (flet ((ends-with (str ch)
           (and (not (null-string-p str)) (char= (elt str (1- (length str))) ch))))
    (let* ((sp (if (ends-with model-namespace #\/) "" "/"))
           (stop-portable (concatenate string model-namespace sp "stop/portable_object"))
           (stop-nocontact (concatenate string model-namespace sp "stop/nocontact"))
           (reset-srv (concatenate string model-namespace sp "lstm/reset")))
      `(progn
         (push (cons (list ,arm ,object-topic) ,reset-srv) *reset-lstm-service-list*)
         (install-lstm-manipulation-intervention ,stop-portable ,arm ,object-topic ,condition-name
                                                 ,groupname ,@init-args)
         (install-lstm-nocontact-intervention ,stop-nocontact ,arm ,object-topic ,condition-name
                                              ,groupname ,@init-args)
         t))))
