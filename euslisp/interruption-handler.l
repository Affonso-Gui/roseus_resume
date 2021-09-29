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
(import 'robot-interface (find-package "ROSEUS_RESUME"))
(in-package "ROSEUS_RESUME")
(export '(on-interruption on-standby on-resume install-interruption-handler
          intervention defintervention define-urgent-intervention
          install-intervention-framework))


;;;;;;;;;;;;;;;;;;;;;;;;;;
;; INTERRUPTION HANDLERS
;;;;;;;;;;;;;;;;;;;;;;;;;;

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
(defmacro install-interruption-handler (robot-interface)
  `(block install-interruption-handler

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
(defvar *intervention-groupname* "roseus_resume.intervention")
(defvar *urgent-intervention-signal* unix::sigurg)
(defvar *monitor-interventions* t)
(defvar *synch-port* (instance sys:synch-memory-port :init))

;; utility
(defun signal-intervention (topic-name condition-name msg)
  (ros::ros-info ";; intervention message received at ~A" topic-name)
  (signals condition-name :ros-msg msg))

(defun urgent-intervention-cb (topic-name condition-name msg)
  (ros::ros-info ";; urgent intervention message received at ~A" topic-name)
  (send *synch-port* :write (list condition-name :ros-msg msg))
  (unix::kill (unix:getpid) *urgent-intervention-signal*))

(defun signal-urgent-intervention (sig code)
  (ros::ros-info ";; signal-urgent-intervention")
  (apply #'signals (send *synch-port* :read)))

(defun spin-intervention-groupname ()
  (ros::ros-info ";; checking for intervention messages...")
  (while *monitor-interventions*
    (ros::spin-once *intervention-groupname*)
    (ros::sleep))
  (ros::ros-warn ";; stop checking for intervention messages..."))

;; installer
(defun install-intervention-framework ()
  ;; create and spin nodehandle in a new thread
  (when (ros::create-nodehandle *intervention-groupname*)
    ;; only execute for the first time
    (sys:make-thread 1)
    (sys:thread-no-wait 'spin-intervention-groupname)
    (unix:signal *urgent-intervention-signal* 'signal-urgent-intervention)

    ;; generate & install default interventions
    (define-urgent-intervention interruption-topic "roseus_resume/interrupt" std_msgs::Empty)
    (define-urgent-intervention resume-topic "roseus_resume/resume" std_msgs::Empty)
    (install-handler interruption-topic
      #'(lambda (c) (signals unix::sigint-received :message "topic interrupt")))
    (install-handler resume-topic #'(lambda (c) (signals unix::sigcont-received)))

    t))

;; macros
(defmacro defintervention (condition-name topic-name message-type &key groupname)
  `(progn
     (defcondition ,condition-name :super intervention)
     ,(if groupname
          `(ros::subscribe ,topic-name ,message-type #'signal-intervention
                           ,topic-name ,condition-name
                           :groupname ,groupname)
          `(ros::subscribe ,topic-name ,message-type #'signal-intervention
                           ,topic-name ,condition-name))))

(defmacro define-urgent-intervention (condition-name topic-name message-type)
  `(progn
     (defcondition ,condition-name :super intervention)
     (ros::subscribe ,topic-name ,message-type #'urgent-intervention-cb
                     ,topic-name ,condition-name
                     :groupname *intervention-groupname*)))
