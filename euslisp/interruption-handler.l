(require :robot-interface "package://pr2eus/robot-interface.l")

;;;;;;;;;;;;;;;;;;;;;;;;;
;; CONTROLLER INTERFACE
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod controller-action-client
 (:action-goal ()
  (send ros::comm-state :action-goal))
 (:current-time ()
  (cond
    ((send ri :simulation-modep)
     (ros::time (/ current-time 1000.0)))
    ((null (send ros::comm-state :action-goal))
     (ros::time 0))
    (t
     (ros::time- last-feedback-msg-stamp
                 (send (send ros::comm-state :action-goal) :header :stamp))))))

(defmethod robot-interface
 (:get-action-status ()
  (send self :spin-once)
  (mapcar
   #'(lambda (ac)
       ;; only interrupt active controllers
       (when (equal (send ac :get-val "SIMPLE-STATE") ros::*simple-goal-state-active*)
         (list (send ac :name) (send ac :action-goal) (send ac :current-time))))
   controller-actions))
 (:interrupt-angle-vector ()
  (send self :cancel-angle-vector))
 (:resume-angle-vector (status &key (scale 1.0) (min-time 1.0))
   (unless (some #'identity status)
     (return-from :resume-angle-vector nil))
   (send *ri* :spin-once) ;; update joint positions
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
                   (joint-names (cdr (assoc :joint-names controller)))
                   (joint-positions
                    (mapcar #'(lambda (j)
                                (send robot (intern (string-upcase j) *keyword-package*)
                                      :ros-joint-angle))
                            joint-names)))
              (assert joint-names "Cannot find :joint-names of controller ~A" controller)
              (mapcar #'cons joint-names joint-positions)))
          (update-offset (start end)
            (dolist (p start)
              (let* ((diff (abs (- (cdr p) (cdr (assoc (car p) end :test #'string=)))))
                     (joint (send robot (intern (string-upcase (car p)) *keyword-package*)))
                     (off (* scale (/ diff (send joint :max-joint-velocity)))))
                (if (> off offset) (setq offset off))))))

       (dolist (s status)
         (when s
           (let* ((name (first s))
                  (goal (send (second s) :goal))
                  (tm (third s))
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
             (update-offset (actual-positions name)
                            (map cons #'cons
                                 (send goal :trajectory :joint_names)
                                 (send (car (send goal :trajectory :points)) :positions))))))

       ;; Check min-time
       (setq offset (max offset min-time))

       (mapc #'(lambda (s ac)
                 (when s
                   (let ((action-goal (send ac :make-goal-instance))
                         (goal (send (second s) :goal))
                         (tm (ros::time-now)))

                     ;; Shift based on time offset
                     (send goal :trajectory :points
                           (mapcar #'(lambda (p) (offset-point p offset))
                                   (send goal :trajectory :points)))

                     ;; Prepare message
                     (send action-goal :header :stamp tm)
                     (send action-goal :header :seq 1)
                     (send action-goal :goal :trajectory (send goal :trajectory))
                     (send action-goal :goal :trajectory :header :stamp tm)
                     (send action-goal :goal :trajectory :header :seq 1)

                     ;; Send goal to controller
                     (send self :spin-once)
                     (send ac :send-goal action-goal))))
             status controller-actions)

       ;; Wait offset before resuming
       (unix:usleep (truncate (* offset 1000000)))
       t))))


;;;;;;;;;;;;;;;;;;;;;;;
;; PACKAGE DEFINITION
;;;;;;;;;;;;;;;;;;;;;;;

(unless (find-package "ROSEUS_RESUME") (make-package "ROSEUS_RESUME"))
(ros::load-ros-manifest "roseus_resume")
(import 'robot-interface (find-package "ROSEUS_RESUME"))
(in-package "ROSEUS_RESUME")
(export '(on-interruption on-standby on-resume install-interruption-handler
          intervention install-intervention-framework defintervention
          register-callable-intervention))


;;;;;;;;;;;;;;;;;;;;;;;;;;
;; INTERRUPTION HANDLERS
;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *namespace* "roseus_resume")

;; conditions
(defcondition on-interruption :slots (status))
(defcondition on-standby :slots (status interruption-instance))
(defcondition on-resume :slots (status))

;; define callback
(defmacro generate-interruption-handler-with-controller-interrupt (robot-interface)
  ;; compile handler to keep the top-level env during the interruption repl
  `(compile
    (defun interruption-handler-with-controller-interrupt (c)
      (let ((action-status
             (handler-case (send ,robot-interface :get-action-status)
               ;; ensure interruption even upon error
               (error (e) (signals on-interruption :status nil)
                      (signals e)))))
        (ros::ros-info ";; action-status: ~A" action-status)
        (ros::ros-info ";; on-interruption...")
        (signals on-interruption :status action-status)
        (ros::ros-info ";; on-standby...")
        (signals on-standby :status action-status
                 :interruption-instance c)
        (ros::ros-info ";; on-resume...")
        (signals on-resume :status action-status)))))

;; install callback
(defmacro install-interruption-handler (robot-interface &key (namespace *namespace*))
  `(block install-interruption-handler
     ;; set interruption signals
     (unless (ros::service-exists "roseus_resume/register_interruption_process")
       (error "Service roseus_resume/register_interruption_process not found. Make sure to execute roslaunch roseus_resume start_monitoring.launch"))

     ;; register_interruption_process fails if called by the second time
     ;; so we do not use assert here
     (unless
         (send
          (ros::service-call
           "roseus_resume/register_interruption_process"
           (instance roseus_resume::RegisterInterruptionProcessRequest :init
                     :pid ,(unix:getpid)
                     :namespace ,namespace))
          :success)
       (return-from install-interruption-handler nil))

     ;; install intervention system
     (install-intervention-framework)

     ;; generate & install interruption handlers
     (generate-interruption-handler-with-controller-interrupt ,robot-interface)
     (install-handler interruption 'interruption-handler-with-controller-interrupt)
     (install-handler on-interruption
                      #'(lambda (c) (send ,robot-interface :interrupt-angle-vector)))
     (install-handler on-standby
                      #'(lambda (c) (invoke-next-handler (send c :interruption-instance))))
     (install-handler on-resume
                      #'(lambda (c) (send ,robot-interface :resume-angle-vector (send c :status))))

     t))


;;;;;;;;;;;;;;;;;
;; INTERVENTION
;;;;;;;;;;;;;;;;;

(defcondition intervention :slots (ros-msg))
(defconstant +intervention-groupname+ "roseus_resume.intervention")
(defun spin-sigurg (condition-instance)
  (declare (ignore condition-instance))
  (ros::wait-for-message 0.2 +intervention-groupname+))

(defun install-intervention-framework ()
  (unless (ros::service-exists "roseus_resume/register_intervention")
    (error "Service roseus_resume/register_intervention not found. Make sure to execute roslaunch roseus_resume start_monitoring.launch"))

  ;; test if already installed
  (when (ros::create-nodehandle +intervention-groupname+)
    ;; setup UNIX signal handler
    (unix:install-signal-handler unix::sigurg unix::sigurg-received)
    (install-handler unix::sigurg-received 'spin-sigurg)
    (install-handler interruption
       #'(lambda (c)
           ;; suppress interventions during repl evaluation
           (handler-bind ((unix::sigurg-received #'(lambda (condition-instance) nil)))
             ;; call default handler
             (invoke-next-handler c))))
    t))

(defmacro defintervention (name &key topic-name (message-type std_msgs::Empty))
  (assert (symbolp name) type-error "symbol expected")
  (flet ((ros-name (sym)
           (string-downcase
            (map string #'(lambda (ch) (if (alphanumericp ch) ch #\_))
                 (symbol-pname sym)))))
    (let ((topic-name (or topic-name (format nil "/~A/~A" *namespace* (ros-name name)))))
      `(progn
         (defcondition ,name :super intervention)
         (ros::subscribe ,topic-name ,message-type
			 `(lambda-closure nil 0 0 (msg)
			    (signals ,',name :ros-msg msg))
                         :groupname +intervention-groupname+)
         (let ((req (instance roseus_resume::RegisterInterventionRequest :init))
               success)
           (send req :pid ,(unix:getpid))
           (send req :topic ,topic-name)
           (send req :type (princ-to-string (send ,message-type :name)))
           (setq success
                 (send (ros::service-call "roseus_resume/register_intervention" req) :success))
           (unless success
             (ros::error "Failed to register intervention ~A at ~A!" ',name ,topic-name))
           ;; return value
           (when success ',name))))))


;; Utility to define and organize multiple callback definitions

(defclass intervention-callback)
(defmethod intervention-callback
  (:init (&rest cbs)
    (dolist (cb cbs)
      (send self :register-callback cb))
    self)
  (:register-callback (cb)
    (eval
     `(install-handler
       ,(intern (symbol-pname cb) "ROSEUS_RESUME")
       ',`(lambda-closure nil 0 0 (c) (send ,self ,cb (send c :ros-msg))))))))

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
     (instance intervention-callback :init ,@(mapcar #'first methods))))
