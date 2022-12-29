
(unless (find-package "ROSEUS_RESUME")
  (load "package://roseus_resume/euslisp/interruption-handler.l"))
(in-package "ROSEUS_RESUME")
(export '(install-lstm-model reset-lstm-models))

(defvar *reset-lstm-service-list* nil)

(defun reset-lstm-models ()
  (dolist (srv *reset-lstm-service-list*)
    (ros::service-call srv (instance std_srvs::TriggerRequest :init))))

(defun ros::unresolve-name (name)
  (substitute #\- #\_ (substitute #\- #\/ (string-trim "/" name))))

(defmacro install-lstm-intervention (topic condition-name predicate
                                     &optional groupname &rest init-args)
  (let ((lstm-condition-name
         (intern (string-upcase (ros::unresolve-name topic)))))
    `(progn
       (defintervention ,lstm-condition-name ,topic std_msgs::Empty
                        :groupname ,groupname)
       (install-handler ,lstm-condition-name
          #'(lambda (c)
              (handler-bind ((,lstm-condition-name #'identity))
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
          (assoc ,object-topic user::*angle-vector-tint :test #'equal))
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
         (push ,reset-srv *reset-lstm-service-list*)
         (install-lstm-manipulation-intervention ,stop-portable ,arm ,object-topic ,condition-name
                                                 ,groupname ,@init-args)
         (install-lstm-nocontact-intervention ,stop-portable ,arm ,object-topic ,condition-name
                                              ,groupname ,@init-args)
         t))))
