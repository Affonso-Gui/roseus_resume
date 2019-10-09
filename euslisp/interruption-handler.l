(ros::load-ros-manifest "roseus_resume")


;;;;;;;;;;;;;;;;;;;;;;;;;
;; CONTROLLER INTERFACE
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod robot-interface
 (:get-action-status ()
  (ros::ros-warn "No actions being monitored")
  nil)
 (:interrupt-angle-vector ()
  (send self :cancel-angle-vector))
 (:resume-angle-vector (status &key (scale 1.0) (min-time 1.0))
   (unless (send status :status) (return-from :resume-angle-vector nil))
   (let ((offset 0.0))
     (labels
         ((offset-point (point tm)
            (send point :time_from_start (ros::time+ (send point :time_from_start) (ros::time tm)))
            point)
          (actual-positions (name)
            (let* ((controller
                    (find name (send self controller-type)
                          :key #'(lambda (x) (cdr (assoc :controller-action x)))
                          :test #'string-equal))
                   (state (cdr (assoc :controller-state controller)))
                   (msg (send self :state (intern (string-upcase state) *keyword-package*))))
              (map cons #'cons (send msg :joint_names) (send msg :actual :positions))))
          (update-offset (start end)
            (dolist (p start)
              (let* ((diff (abs (- (cdr p) (cdr (assoc (car p) end :test #'string=)))))
                     (joint (send robot (intern (string-upcase (car p)) *keyword-package*)))
                     (off (* scale (/ diff (send joint :max-joint-velocity)))))
                (if (> off offset) (setq offset off))))))

       (dolist (s (send status :status))
         (let* ((goal (send s :goal))
                (feedback (send s :feedback))
                (tm (send feedback :actual :time_from_start))
                (last-point (car (last (send goal :trajectory :points)))))

           ;; Remove completed steps
           (send goal :trajectory :points
                 (remove-if #'(lambda (p) (ros::time< (send p :time_from_start) tm))
                            (send goal :trajectory :points)))

           ;; Ensure last waypoint
           (unless (send goal :trajectory :points)
             (send last-point :time_from_start tm)
             (send goal :trajectory :points (list last-point)))

           ;; Shift based on time_from_start
           (send goal :trajectory :points
                 (mapcar #'(lambda (p) (offset-point p (- (send tm :to-sec))))
                         (send goal :trajectory :points)))

           ;; Update offset
           (update-offset (actual-positions (send s :name))
                          (map cons #'cons
                               (send goal :trajectory :joint_names)
                               (send (car (send goal :trajectory :points)) :positions)))))

       ;; Check min-time
       (setq offset (max offset min-time))

       (dolist (s (send status :status))
         (let* ((action (find (send s :name) controller-actions
                              :key #'(lambda (x) (send x :name))
                              :test #'string-equal))
                (action-goal (send action :make-goal-instance))
                (tm (ros::time-now)))

           ;; Shift based on time offset
           (send s :goal :trajectory :points
                 (mapcar #'(lambda (p) (offset-point p offset))
                         (send s :goal :trajectory :points)))

           ;; Prepare message
           (send action-goal :header :stamp tm)
           (send action-goal :header :seq 1)
           (send action-goal :goal :trajectory (send s :goal :trajectory))
           (send action-goal :goal :trajectory :header :stamp tm)
           (send action-goal :goal :trajectory :header :seq 1)

           ;; Send goal to controller
           (send self :spin-once)
           (send action :send-goal action-goal)))

       ;; Wait offset before resuming
       (unix:usleep (truncate (* offset 1000000)))))))


;;;;;;;;;;;;;;;;;;;;;;;
;; PACKAGE DEFINITION
;;;;;;;;;;;;;;;;;;;;;;;

(unless (find-package "ROSEUS_RESUME") (make-package "ROSEUS_RESUME"))
(import 'robot-interface (find-package "ROSEUS_RESUME"))
(in-package "ROSEUS_RESUME")
(export '(on-interruption on-standby on-resume install-interruption-handler
          intervention defintervention register-callable-intervention))


;;;;;;;;;;;;;;;;;;;;;;;;;;
;; INTERRUPTION HANDLERS
;;;;;;;;;;;;;;;;;;;;;;;;;;

;; conditions
(defcondition on-interruption :slots (status) :super interruption-event)
(defcondition on-standby :slots (status) :super interruption-event)
(defcondition on-resume :slots (status) :super interruption-event)

;; define callback
(defmacro generate-interruption-handler-with-controller-interrupt (robot-interface service-name)
  ;; compile handler to keep the top-level env during the interruption repl
  `(progn
     (defmethod robot-interface
       (:get-action-status ()
        (ros::service-call ,service-name
          (instance roseus_resume::GetControllerActionStatusRequest :init))))
     (compile
      (defun interruption-handler-with-controller-interrupt (c)
        (let ((action-status (send ,robot-interface :get-action-status)))
          (signals on-interruption :status action-status)
          (signals on-standby :msg "keyboard interrupt" :status action-status)
          (signals on-resume :status action-status))))))

;; install callback
(defmacro install-interruption-handler (robot-interface &key (namespace "roseus_resume"))
  (let ((service-name "/roseus_resume/get_status"))
    `(progn
       ;; set interruption signals
       (unix:system ,(format nil "rosrun roseus_resume stop_process ~A -ns ~A ~&"
                             (unix:getpid) namespace))

       ;; monitor controllers state
       (unless (or (ros::service-exists ,service-name)
                   (send ,robot-interface :simulation-modep))
         (unix:system
          (with-output-to-string (s)
            (format s "rosrun roseus_resume monitor_controller")
            (dolist (ca (send ,robot-interface :get-val "CONTROLLER-ACTIONS"))
              (format s " -c ~A" (send ca :name)))
            (format s " &"))))

       ;; generate & install interruption handlers
       (generate-interruption-handler-with-controller-interrupt ,robot-interface ,service-name)
       (install-handler interruption-event 'interruption-handler-with-controller-interrupt)
       (install-handler on-interruption
         #'(lambda (c) (send ,robot-interface :interrupt-angle-vector)))
       (install-handler on-standby
         #'lisp::interruption-handler)
       (install-handler on-resume
         #'(lambda (c) (send ,robot-interface :resume-angle-vector (send c :status)))))))


;;;;;;;;;;;;;;;;;
;; INTERVENTION
;;;;;;;;;;;;;;;;;

(defcondition intervention :slots (request))
(defconstant *intervention-groupname* "intervention")
(ros::create-nodehandle *intervention-groupname*)
(defun ros::spin-sigusr1 (sig code) (ros::wait-for-message 0.5 *intervention-groupname*))
(unix:signal unix::sigusr1 'ros::spin-sigusr1)

(defmacro defintervention (name &key topic-name (message-type std_msgs::Empty))
  (flet ((ros-name (sym)
           (string-downcase
            (map string #'(lambda (ch) (if (alphanumericp ch) ch #\_))
                 (symbol-pname sym)))))
    (let ((topic-name (or topic-name (format nil "/roseus_resume/~A" (ros-name name))))
          (srv-name "/roseus_resume/register_intervention"))
      `(progn
         (unless (boundp ',name)
           (defcondition ,name :super intervention))
         (ros::subscribe ,topic-name ,message-type
                         #'(lambda (msg) (signals ,name :request msg))
                         :groupname *intervention-groupname*)
         (ros::wait-for-service ,srv-name)
         (ros::ros-info ,(format nil "Listener installed on ~S" topic-name))
         (ros::service-call ,srv-name
                            (instance roseus_resume::RegisterInterventionRequest :init :topic ,topic-name))))))


;; Utility to define and organize multiple callback definitions

(defclass intervention-callback)
(defmethod intervention-callback
  (:register-callbacks (&rest cbs)
    (dolist (cb cbs)
      (eval
       `(install-handler
         ,(intern (symbol-pname cb) "ROSEUS_RESUME")
         ',`(lambda-closure nil 0 0 (c) (send ,self ,cb (send c :request))))))))

(defmacro register-callable-intervention (&rest methods)
  "(&rest (selector lambda-list msg-type &rest body))"
  `(progn
     ;; define interventions
     ,@(mapcar #'(lambda (m)
                   (let ((selector (first m))
                         (msg-type (third m)))
                     `(defintervention ,(intern (symbol-pname selector) "ROSEUS_RESUME")
                          :message-type ,msg-type)))
               methods)
     ;; define callback methods
     (defmethod intervention-callback
         ,@(mapcar #'(lambda (m)
                       (let ((selector (first m))
                             (lambda-list (second m))
                             ;; (msg-type (third m))
                             (body (cdddr m)))
                         `(,selector ,lambda-list ,@body)))
                   methods))
     ;; initialize instance
     (instance intervention-callback :register-callbacks ,@(mapcar #'first methods))))
