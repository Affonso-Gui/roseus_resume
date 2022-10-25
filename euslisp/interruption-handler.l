(require :robot-interface "package://pr2eus/robot-interface.l")
(unless (find-package "ROSEUS_RESUME") (make-package "ROSEUS_RESUME"))

;;;;;;;;;;;;;;;;;;;;;;;;;
;; CONTROLLER INTERFACE
;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod ros::simple-action-client
 (:current-time ()
  (cond
    ((null (send ros::comm-state :action-goal))
     (ros::time 0))
    (t
     (ros::time- (ros::time-now)
                 (send (send ros::comm-state :action-goal) :header :stamp))))))

(defmethod controller-action-client
 (:current-time ()
  (cond
    ((send ri :simulation-modep)
     (ros::time (/ (or current-time 0.0) 1000.0)))
    ((null (send ros::comm-state :action-goal))
     (ros::time 0))
    (t
     ;; use time measured from last feedback instead
     (ros::time- last-feedback-msg-stamp
                 (send (send ros::comm-state :action-goal) :header :stamp))))))


(defclass roseus_resume::action-snapshot
  :super propertied-object
  :slots (ac goal time))
(defclass roseus_resume::controller-action-snapshot
    :super roseus_resume::action-snapshot)
(defmethod roseus_resume::action-snapshot
  (:init (action-client)
    (setq ac action-client)
    (setq goal (send (send ac :comm-state) :action-goal))
    (setq time (send ac :current-time)))
  (:name () (send ac :name))
  (:action-client () ac)
  (:goal () (send goal :goal))
  (:time () time))

(defmethod robot-interface
 (:get-action-status ()
  (send self :spin-once)
  (let (acc)
    (labels ((check-active (ac)
               (= (send ac :get-state) actionlib_msgs::GoalStatus::*active*))
             (make-action-snapshot (ac)
               (if (and (derivedp ac controller-action-client)
                        (derivedp (ac . ros::action-spec)
                                  control_msgs::followjointtrajectoryaction))
                   (instance roseus_resume::controller-action-snapshot :init ac)
                   (instance roseus_resume::action-snapshot :init ac)))
             (get-actions-from-slots ()
               (let (action-acc)
                 (labels ((maybe-push-action (value)
                            (cond
                              ((derivedp value ros::simple-action-client)
                               (push value action-acc))
                              ((consp value)
                               (mapc #'maybe-push-action value)))))
                   (dolist (slot (send self :slots))
                     (maybe-push-action (cdr slot)))
                   (nreverse action-acc)))))
      (dolist (ac (get-actions-from-slots))
        ;; only interrupt active controllers
        (when (check-active ac)
          (push (make-action-snapshot ac) acc)))
      (dolist (ac (get self :additional-controllers))
        ;; only interrupt active controllers
        (when (check-active ac)
          (push (make-action-snapshot ac) acc)))
      (nreverse acc))))
 (:interrupt-angle-vector (&optional status)
  (send self :cancel-angle-vector)
  ;; `controller-actions' are stopped with cancel-angle-vector
  ;; stop other actions by directly sending a cancel request
  (dolist (snapshot
            (remove-if #'(lambda (s) (find (send s :action-client) controller-actions)) status))
    (send (send snapshot :action-client) :cancel-goal)))
 (:resume-angle-vector (status &key (scale 1.0) (min-time 1.0))
   (unless (some #'identity status)
     (return-from :resume-angle-vector nil))
   (send self :spin-once) ;; update joint positions
   (let ((offset 0.0)
         (controller-action-status
          (remove-if-not #'(lambda (val) (derivedp val roseus_resume::controller-action-snapshot))
                         status))
         (simple-action-status
          (remove-if #'(lambda (val) (derivedp val roseus_resume::controller-action-snapshot))
                     status)))
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

       (dolist (s controller-action-status)
         (let* ((goal (send s :goal))
                (tm (send s :time))
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

       ;; Shift based on time offset
       (dolist (s controller-action-status)
         (let ((goal (send s :goal)))
           (send goal :trajectory :points
                 (mapcar #'(lambda (p) (offset-point p offset))
                         (send goal :trajectory :points)))))

       ;; Send commands
       (dolist (s status)
         (let* ((ac (send s :action-client))
                (action-goal (send ac :make-goal-instance))
                (goal (send s :goal))
                (tm (ros::time-now)))

           ;; Prepare message
           (send action-goal :header :stamp tm)
           (send action-goal :header :seq 1)
           (if (derivedp s roseus_resume::controller-action-snapshot)
               (progn
                 (send action-goal :goal :trajectory (send goal :trajectory))
                 (send action-goal :goal :trajectory :header :stamp tm)
                 (send action-goal :goal :trajectory :header :seq 1))
               (send action-goal :goal goal))

           ;; Send goal to controller
           (send self :spin-once)
           (send ac :send-goal action-goal)))

       ;; Wait offset before resuming
       (when controller-action-status
         (unix:usleep (truncate (* offset 1000000))))
       t))))


;;;;;;;;;;;;;;;;;;;;;;;
;; PACKAGE DEFINITION
;;;;;;;;;;;;;;;;;;;;;;;

(import 'robot-interface (find-package "ROSEUS_RESUME"))
(in-package "ROSEUS_RESUME")
(export '(on-interruption on-standby on-resume install-interruption-handler
          intervention defintervention define-urgent-intervention
          install-default-intervention install-urgent-intervention-framework))


;;;;;;;;;;;;;;;;;;;;;;;;;;
;; INTERRUPTION HANDLERS
;;;;;;;;;;;;;;;;;;;;;;;;;;

;; conditions
(defcondition on-interruption :slots (status))
(defcondition on-standby :slots (status interruption-instance))
(defcondition on-resume :slots (status))

;; define callback
(defun generate-interruption-handler-with-controller-interrupt (ri)
  ;; compile handler to keep the top-level env during the interruption repl
  (compile
   (eval
    `(defun interruption-handler-with-controller-interrupt (c)
       (let ((action-status
              (handler-case (send ,ri :get-action-status)
                ;; ensure interruption even upon error
                (error (e) (signals on-interruption :status nil)
                       (euserror e)))))
         (ros::ros-info ";; action-status: ~A" action-status)
         (ros::ros-info ";; on-interruption...")
         (signals on-interruption :status action-status)
         (ros::ros-info ";; on-standby...")
         (signals on-standby :status action-status
                  :interruption-instance c)
         (ros::ros-info ";; on-resume...")
         (signals on-resume :status action-status))))))

;; install callback
(defun install-interruption-handler (ri &rest additional-controllers)
  ;; store additional controllers
  (let (acc)
    (dolist (controller additional-controllers)
      (if (derivedp controller ros::simple-action-client)
          (progn
            (ros::ros-info "Registering additional controller: ~S" controller)
            (push controller acc))
          (progn
            (ros::ros-warn "Additional controllers must be derived from ros::simple-action-client class: ~S" controller)
            (ros::ros-warn "Skipping ~S..." controller))))
    (setf (get ri :additional-controllers)
          (remove-duplicates (append (get ri :additional-controllers) (nreverse acc)))))
  ;; generate & install interruption handlers
  (unless (fboundp 'interruption-handler-with-controller-interrupt)
    (ros::ros-info "Installing interruption handler...")
    (generate-interruption-handler-with-controller-interrupt ri)
    (install-handler interruption 'interruption-handler-with-controller-interrupt)
    (install-handler on-interruption
                     #'(lambda (c) (send ri :interrupt-angle-vector (send c :status))))
    (install-handler on-standby
                     #'(lambda (c) (invoke-next-handler (send c :interruption-instance))))
    (install-handler on-resume
                     #'(lambda (c) (send ri :resume-angle-vector (send c :status))))
    (ros::ros-info "Interruption handler installed.")
    t))


;;;;;;;;;;;;;;;;;
;; INTERVENTION
;;;;;;;;;;;;;;;;;

(defcondition intervention :slots (ros-msg))
(defvar *intervention-groupname* "roseus_resume.intervention")
(defvar *default-intervention-groupname* "roseus_resume.default_intervention")
(defvar *urgent-intervention-signal* unix::sigurg)
(defvar *monitor-interventions* t)
(defvar *synch-port* (instance sys:synch-memory-port :init))

;; utility
(defun signal-intervention (topic-name condition-name msg)
  (ros::ros-info ";; intervention message received at ~A" topic-name)
  (if (derivedp (instance condition-name) intervention)
      (signals condition-name :ros-msg msg)
      (signals condition-name)))

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

;; default intervention installer
(defun install-default-intervention (robot-interface &key
                                       (interrupt-topic "/roseus_resume/interrupt")
                                       (resume-topic "/roseus_resume/resume")
                                       (top-selector-interval))
  ;; Spin topics as a timer-job, meaning that they are only invoked during idle time.
  ;; For interrupting actions during their execution, try registering interventions
  ;; directly to the robot-interface's groupname `(*ri* . groupname)`, or using
  ;; the `install-urgent-intervention-framework' instead
  (when (ros::create-nodehandle *default-intervention-groupname*)
    ;; only execute for the first time
    (if (or (not (eq (class *standard-input*) file-stream))
            (unix:isatty *standard-input*))
        ;; check for resumptions as a timer-job if process is atty
        (progn
          (if top-selector-interval
            (setq *top-selector-interval* top-selector-interval))
          (setq *timer-job* (cons #'(lambda () (ros::spin-once *default-intervention-groupname*))
                                  *timer-job*)))
        ;; check for resumptions in a custom handler if process is not atty
        (install-handler interruption
            #'(lambda (c)
                (lisp::print-error-message c)
                (ros::ros-info "Process interrupted by user. Send a message to ~A to continue"
                               resume-topic)
                (handler-case
                    (while (ros::ok)
                      (ros::spin-once *default-intervention-groupname*)
                      (ros::sleep))
                  (unix::sigcont-received () ))))))

  ;; spin interruptions on the robot and resumptions as a timer-job or handler
  (defintervention interruption interrupt-topic std_msgs::Empty
                   :groupname (send robot-interface :get-val "GROUPNAME"))
  (defintervention unix::sigcont-received resume-topic std_msgs::Empty
                   :groupname *default-intervention-groupname*)
  t)

(defun install-urgent-intervention-framework ()
  ;; Dispatch unix signals when a topic message is received,
  ;; ensuring immediate interruption at any execution point
  (when (ros::create-nodehandle *intervention-groupname*)
    ;; create and spin nodehandle in a new thread
    ;; only execute for the first time
    (warn ";; Urgent interventions are experimental! Handle with care~%")
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
     (unless (boundp ',condition-name)
       (defcondition ,condition-name :super intervention))
     (unless (and (classp ,condition-name) (derivedp (instance ,condition-name) condition))
       (error value-error "condition class expected in ~A(~S)" ',condition-name ,condition-name))
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
