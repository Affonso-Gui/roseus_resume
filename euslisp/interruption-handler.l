;; ;; don't explicitly load to avoid introducing cyclic dependencies
;; ;; instead, let if fail when robot-interface has not been loaded
;; (require :robot-interface "package://pr2eus/robot-interface.l")

(require :logger "lib/llib/logger")

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
(defclass roseus_resume::interruption-snapshot
  :super propertied-object
  :slots (action-status robot-state checkpoint-list tint gripper-tint current-state))

(defmethod roseus_resume::action-snapshot
  (:init (action-client)
    (setq ac action-client)
    (setq goal (send (send ac :comm-state) :action-goal))
    (setq time (send ac :current-time)))
  (:name () (send ac :name))
  (:action-client () ac)
  (:goal () (send goal :goal))
  (:time () time))

(defmethod roseus_resume::controller-action-snapshot
  (:angle-vector (robot-state)
    (let ((cacts (find (get ac :name) (send (ac . ri) :default-controller)
                       :key #'(lambda (x) (cdr (assoc :controller-action x)))
                       :test #'string=)))
      (when cacts
        (let ((controller-joint-names (cdr (assoc :joint-names cacts)))
              (joint-names (cdr (assoc :name robot-state)))
              (joint-positions (cdr (assoc :position robot-state))))
          (labels ((ros-joint-angle (nm val)
                     (let ((joint (send ((ac . ri) . robot)
                                        (intern (string-upcase nm) *keyword-package*))))
                       (if (derivedp joint rotational-joint)
                           (rad2deg val)
                           (* val 1000))))
                   (joint-angle (nm)
                     (ros-joint-angle nm
                      (elt joint-positions (position nm joint-names :test #'string=)))))
            (map float-vector #'joint-angle controller-joint-names)))))))

(defmethod roseus_resume::interruption-snapshot
  (:init (status robot-st &key (angle-vector-tint *angle-vector-tint*)
                               ((:gripper-tint gtint) *gripper-tint*)
                               (motion-checkpoint-list *motion-checkpoint-list*))
    (setq action-status status)
    (setq robot-state robot-st)
    (setq checkpoint-list motion-checkpoint-list)
    (setq tint angle-vector-tint)
    (setq gripper-tint gtint)
    self)
  (:start-monitoring (groupname)
    (dolist (topic-value (send self :get-tint))
      (let ((topic (car topic-value))
            (value (cdr topic-value)))
        (if (not (derivedp value ros::object)) (error value-error "ros::object expected"))
        (setq current-state (acons topic nil current-state))
        (ros::subscribe (concatenate string "@" topic) (class value)
                        #'(lambda (msg)
                            (setf (cdr (assoc topic current-state :test #'string=)) msg)
                            (signals roseus_resume::environment-state-update))
                        :groupname groupname))))
  (:stop-monitoring ()
    (dolist (val current-state)
      (ros::unsubscribe (concatenate string "@" (car val)))))
  (:action-status () action-status)
  (:robot-state () robot-state)
  (:current-state () current-state)
  (:checkpoint-list () checkpoint-list)
  (:get-tint ()
    ;; only return topic-value pairs
    (mapcar #'(lambda (x) (cons (car x) (send (cdr x) :message)))
            (remove-if-not
             #'(lambda (x) (and (stringp (car x)) (derivedp (cdr x) ros::ros-tint)))
             tint)))
  (:get-gripper-tint () gripper-tint))

(defmethod robot-interface
 (:robot-state () robot-state)
 (:get-action-status ()
  (let (acc)
    (labels ((check-active (ac)
               (send ac :spin-once)
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
   ;; controller-actions are stopped with `:cancel-angle-vector', to ensure a quick stop
   ;; for more smooth stopping, use `:stop-motion' instead
   (send self :cancel-angle-vector))
 (:interrupt-additional-controllers (status)
   ;; stop other actions by directly sending a cancel request
   (if (derivedp status roseus_resume::interruption-snapshot)
       (setq status (send status :action-status)))
   (dolist (snapshot
             (remove-if #'(lambda (s) (find (send s :action-client) controller-actions)) status))
     (send (send snapshot :action-client) :cancel-goal)))
 (:resume-angle-vector (status &key
                               (scale roseus_resume::*resume-time-scale*)
                               (min-time roseus_resume::*resume-min-time*))
   (if (derivedp status roseus_resume::interruption-snapshot)
       (setq status (send status :action-status)))
   (unless (some #'identity status)
     (return-from :resume-angle-vector nil))
   (send self :spin-once) ;; update joint positions
   (let ((offset 0.0)
         (controller-action-status
          (remove-if-not #'(lambda (val) (derivedp val roseus_resume::controller-action-snapshot))
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
       (dolist (s controller-action-status)
         (let* ((ac (send s :action-client))
                (action-goal (send ac :make-goal-instance))
                (goal (send s :goal))
                (tm (ros::time-now)))

           ;; Prepare message
           (send action-goal :header :stamp tm)
           (send action-goal :header :seq 1)
           (send action-goal :goal :trajectory (send goal :trajectory))
           (send action-goal :goal :trajectory :header :stamp tm)
           (send action-goal :goal :trajectory :header :seq 1)

           ;; Send goal to controller
           (send ac :spin-once)
           (send ac :send-goal action-goal)))

       ;; Wait offset before resuming
       (when controller-action-status
         (unix:usleep (truncate (* offset 1000000))))
       t)))
 (:resume-additional-controllers (status)
   (if (derivedp status roseus_resume::interruption-snapshot)
       (setq status (send status :action-status)))
   ;; Send commands
   (dolist (s status)
     ;; skip controller actions
     (when (and (derivedp s roseus_resume::action-snapshot)
                (not (derivedp s roseus_resume::controller-action-snapshot)))
       (let* ((ac (send s :action-client))
              (action-goal (send ac :make-goal-instance))
              (goal (send s :goal))
              (tm (ros::time-now)))

         ;; Prepare message
         (send action-goal :header :stamp tm)
         (send action-goal :header :seq 1)
         (send action-goal :goal goal)

         ;; Send goal to controller
         (send ac :spin-once)
         (send ac :send-goal action-goal))))))


;;;;;;;;;;;;;;;;;;;;;;;
;; RECURSIVE EPS=
;;;;;;;;;;;;;;;;;;;;;;;

(in-package "GEOMETRY")
(export '(*epsilon-alist* *eps-rec=-last-argument* eps-rec=))

(defvar *epsilon-alist* (list (cons coordinates (list 5.0 (deg2rad 5.0)))))
(defvar *eps-rec=-last-argument* nil
  "last arguments evaluated during an eps-rec= call")

(defun eps-rec= (m n &optional (eps *epsilon*) (eps-rot eps) &key test)
  (setq *eps-rec=-last-argument* (list m n))
  (if (and test (funcall test m n))
      (return-from eps-rec= t))
  (if (not (eq (class m) (class n)))
      (return-from eps-rec= nil))
  (if (assoc (class m) *epsilon-alist*)
      (multiple-value-setq (eps eps-rot) (cdr (assoc (class m) *epsilon-alist*))))
  (cond
    ((numberp m)
     ;; avoid eps= since it is optimized for float
     ;; https://github.com/euslisp/EusLisp/issues/406
     (< (abs (- m n)) eps))
    ((symbolp m) (eql m n))
    ((stringp m) (string-equal m n))
    ((vectorp m) (eps-v= m n eps))
    ((coordinates-p m) (eps-coords= m n eps eps-rot))
    ((and (consp m) (not (listp (cdr m))))
     (and (eps-rec= (car m) (car n) eps eps-rot :test test)
          (eps-rec= (cdr m) (cdr n) eps eps-rot :test test)))
    ((consp m)
     (if (not (= (length m) (length n)))
         (return-from eps-rec= nil))
     (dotimes (i (length m))
       (if (not (eps-rec= (nth i m) (nth i n) eps eps-rot :test test))
           (return-from eps-rec= nil)))
     t)
    ((derivedp m std_msgs::Header)
     ;; never check headers
     t)
    ((derivedp m ros::Time)
     ;; never check time
     t)
    ((derivedp m geometry_msgs::Pose)
     ;; convert to coordinates
     (eps-rec= (ros::tf-pose->coords m) (ros::tf-pose->coords n) eps eps-rot :test test))
    (t
     (catch :return
       (map nil #'(lambda (a b)
                    (if (and (not (memq (car a) '(plist tint-vector ros::_connection-header))))
                        (if (not (eps-rec= (cdr a) (cdr b) eps eps-rot :test test))
                            (throw :return nil))))
            (send m :slots)
            (send n :slots))
       t))))


;;;;;;;;;;;;;;;;;;;;;;;
;; PACKAGE DEFINITION
;;;;;;;;;;;;;;;;;;;;;;;

(import 'robot-interface (find-package "ROSEUS_RESUME"))
(in-package "ROSEUS_RESUME")
(export '(on-interruption on-standby on-resume install-interruption-handler
          *current-status* *resume-time-scale* *resume-min-time*
          environment-state-update robot-state-eps=
          check-tint-collision check-state find-motion-checkpoint
          intervention defintervention define-urgent-intervention
          install-default-intervention install-urgent-intervention-framework))


;;;;;;;;;;;;;;;;;;;;;;;;;;
;; INTERRUPTION HANDLERS
;;;;;;;;;;;;;;;;;;;;;;;;;;

;; status object
(defvar *current-status* nil
  "Snapshot of interrupted state")
(defvar *resume-time-scale* 1.0
  "Scale used to resume interrupted motions")
(defvar *resume-min-time* 1.0
  "Minimum resumption time used to resume interrupted motions")

;; conditions
(defcondition on-interruption :slots (status))
(defcondition on-standby :slots (status interruption-instance))
(defcondition on-resume :slots (status))
(defcondition environment-state-update)

;; define callback
(defun generate-interruption-handler-with-controller-interrupt (ri)
  ;; compile handler to keep the top-level env during the interruption repl
  (compile
   (eval
    `(defun interruption-handler-with-controller-interrupt (c)
       (let ((*current-status*))
         (lisp::atomic
             (setq *current-status*
                   (instance interruption-snapshot :init
                             (send ,ri :get-action-status)
                             (copy-object (send ,ri :robot-state))))
           ;; ensure interruption even upon error
           (error (e) (signals on-interruption :status nil)
                  (euserror e))
           ;; delay other executions
           (condition ()))
         (unwind-protect
              (progn
                (send *current-status* :start-monitoring (,ri . user::groupname))
                (ros::ros-info ";; action-status: ~A"
                               (and *current-status* (send *current-status* :action-status)))
                (ros::ros-info ";; on-interruption...")
                (signals on-interruption :status *current-status*)
                (ros::ros-info ";; on-standby...")
                (signals on-standby :status *current-status*
                         :interruption-instance c)
                (ros::ros-info ";; on-resume...")
                (signals on-resume :status *current-status*))
           (send *current-status* :stop-monitoring)))))))

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
                     #'(lambda (c)
                         (send ri :interrupt-angle-vector (send c :status))
                         (send ri :interrupt-additional-controllers (send c :status))))
    (install-handler on-standby
                     #'(lambda (c) (invoke-next-handler (send c :interruption-instance))))
    (install-handler on-resume
                     #'(lambda (c)
                         (send ri :resume-additional-controllers (send c :status))
                         (send ri :resume-angle-vector (send c :status))))
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


;;;;;;;;;;;;;;;;;
;; STATE CHECK
;;;;;;;;;;;;;;;;;

(use-package "GEOMETRY")

(defun robot-state-eps= (ri prev-robot-state current-robot-state &optional (eps *epsilon*)
                         &key eps-dict blacklist whitelist)
  ;; eps-rec= without joint-angle related, and more custom parameters
  (setq *eps-rec=-last-argument* nil)
  (let ((joint-state-keys '(:position :effort :velocity))
        (controller-state-keys
         (mapcar #'(lambda (x) (intern (string-upcase (cdr (assoc :controller-state x)))
                                       *keyword-package*))
                 (send ri :default-controller))))
    (dolist (rs current-robot-state)
      (let ((key (car rs)))
        (when (and (if whitelist (memq key whitelist) t)
                   (if blacklist (not (memq key blacklist)) t)
                   (not (memq key joint-state-keys))
                   (not (memq key controller-state-keys)))
          (let* ((prev-rs (assoc key prev-robot-state))
                 (eps (or (cdr (assoc key eps-dict)) eps))
                 (res (eps-rec= (cdr prev-rs) (cdr rs) eps)))
            (setq *eps-rec=-last-argument* (list prev-rs rs))
            (if (not res)
                (return-from robot-state-eps= nil))))))
    t))

(defun check-tint-collision (value collision-model &optional (mode :any)
                             &key (collision-range 50.0))
  ;; check collision with coordinate, geometry_msg::Pose, or geometry_msgs::PoseArray
  (assert (or (eq mode :any) (eq mode :all)) "mode should be :any or :all")
  (flet
      ((list-coords (a)
         (cond
           ((coordinates-p a) (list a))
           ((derivedp a geometry_msgs::Pose) (list (ros::tf-pose->coords a)))
           ((derivedp a geometry_msgs::PoseArray)
            (mapcar #'ros::tf-pose->coords (send a :poses)))
           (t nil))))
    (let ((c-list (list-coords value))
          cube)
      (if (null c-list) (return-from check-tint-collision nil))
      (if (numberp collision-range)
          (setq cube (make-cube collision-range collision-range collision-range))
          (setq cube collision-range))
      (dolist (c c-list)
        (send cube :newcoords c)
        (if (= (user::collision-check collision-model cube) 1)
            (if (eq mode :any) (return-from check-tint-collision t))
            (if (eq mode :all) (return-from check-tint-collision nil))))
      (case mode
        (:any nil)
        (:all t)))))

(defun check-state (ri &key (interrupt-state *current-status*)
                         (environment-state-eps *epsilon*)
                         (angle-vector-eps (* 1000 *epsilon*))
                         (robot-state-eps *epsilon*)
                         robot-state-blacklist
                         robot-state-whitelist
                         robot-state-eps-dict
                         (collision-model (ri . user::robot))
                         (collision-range 50.0))
  ;; check if the interrupted state is similar to the current state
  (if (null interrupt-state)
      (return-from check-state nil))
  (if (not (derivedp interrupt-state interruption-snapshot))
      (error type-error "interruption-snapshot expected"))
  (if (and (null (send interrupt-state :current-state))
           (send interrupt-state :get-tint))
      (error "interruption has environment dependencies, but monitoring hasn't been started!"))
  (send ri :spin-once)

  (labels ((test-occlusion (a b)
             (when (and collision-model
                        (not b)
                        (check-tint-collision a collision-model :all
                                              :collision-range collision-range))
               (ros::ros-info ";; Collision Detected")
               t)))

    ;; check environment state
    (when (not (eps-rec= (send interrupt-state :get-tint)
                         (send interrupt-state :current-state)
                         environment-state-eps
                         environment-state-eps
                         :test #'test-occlusion))
      (ros::ros-info ";; Environment State Mismatch at: ~A" *eps-rec=-last-argument*)
      (log:log-debug "INTERRUPTION STATE: ~A" (send interrupt-state :get-tint))
      (if (<= log::*logger-level* (log::get-level :debug))
          (map nil #'(lambda (x) (print (car x)) (user::print-ros-msg (cdr x)))
               (send interrupt-state :get-tint)))
      (log:log-debug "CURRENT STATE: ~A" (send interrupt-state :current-state))
      (if (<= log::*logger-level* (log::get-level :debug))
          (map nil #'(lambda (x) (print (car x)) (user::print-ros-msg (cdr x)))
               (send interrupt-state :current-state)))
      (return-from check-state nil))
    ;; check robot state
    (let* ((current-robot-state (send ri :robot-state))
           (interrupt-robot-state (send interrupt-state :robot-state)))
      ;; joint angles
      (dolist (ac (send interrupt-state :action-status))
        (when (and (derivedp ac controller-action-snapshot)
                   (not (eps-rec= (send ac :angle-vector interrupt-robot-state)
                                  (send ac :angle-vector current-robot-state)
                                  angle-vector-eps)))
          (ros::ros-info ";; joint angle mismatch at: ~a" (send ac :name))
          (log:log-debug "interrupt angle: ~a" (send ac :angle-vector interrupt-robot-state))
          (log:log-debug "current angle: ~a" (send ac :angle-vector current-robot-state))
          (return-from check-state nil)))
      ;; other states
      (when (not (robot-state-eps= ri interrupt-robot-state current-robot-state robot-state-eps
                                   :eps-dict robot-state-eps-dict
                                   :blacklist robot-state-blacklist
                                   :whitelist robot-state-whitelist))
        (ros::ros-info ";; Robot State Mismatch at: ~A" (caar *eps-rec=-last-argument*))
        (log:log-debug "INTERRUPT STATE: ~A" (cdr (first *eps-rec=-last-argument*)))
        (log:log-debug "CURRENT STATE: ~A" (cdr (second *eps-rec=-last-argument*)))
        (return-from check-state nil)))
    t))

(defun find-emark (ri &key (interrupt-state *current-status*)
                        (environment-state-eps *epsilon*)
                        (gripper-state-eps *epsilon*))
  ;; find the environment state marking which is different from interruption state
  (if (not (derivedp interrupt-state interruption-snapshot))
      (error type-error "interruption-snapshot expected"))
  (send ri :spin-once)

  (let ((tint (send interrupt-state :get-tint))
        (gripper-tint (send interrupt-state :get-gripper-tint)))
    (when gripper-tint
      (let* ((current-robot-state (send ri :robot-state))
             (interrupt-robot-state (send interrupt-state :robot-state)))
        (dolist (gtint gripper-tint)
          (let* ((limb (car gtint))
                 (mark (cdr gtint))
                 (key (intern (format nil "~A-POSITION" (string limb)) *keyword-package*)))
            (if (not (assoc key interrupt-robot-state))
                (error value-error "Could not find ~A in interrupted robot state" key))
            (if (not (assoc key current-robot-state))
                (error value-error "Could not find ~A in current robot state" key))
            (when (not (eps-rec= (assoc key interrupt-robot-state)
                                 (assoc key current-robot-state)
                                 gripper-state-eps))
              (ros::ros-info ";; Gripper State Mismatch: ~A ~A" limb (car mark))
              (return-from find-emark (car mark)))))))
    (when tint
      (let ((current-state (send interrupt-state :current-state)))
        (dolist (mark tint)
          (let* ((topic (car mark))
                 (current-mark (assoc topic current-state :test #'string=)))
            (if (not current-mark)
                (error value-error "Could not find topic ~A in current state" topic))
            (when (not (eps-rec= mark current-mark environment-state-eps))
              (ros::ros-info ";; Environment State Mismatch: ~A" topic)
              (return-from find-emark topic))))))
    (ros::ros-info ";; No Mismatch found")
    nil))

(defun find-motion-checkpoint (ri &rest args
                                  &key (interrupt-state *current-status*)
                                  &allow-other-keys)
  ;; find the next checkpoint suitable for resumption
  ;; returns: checkpoint object, motion intent, initial condition
  (let ((emark (apply #'find-emark ri :interrupt-state interrupt-state args))
        (checkpoint-list (send interrupt-state :checkpoint-list)))
    (dotimes (i (length checkpoint-list))
      (let* ((ckpt (nth i checkpoint-list))
             (intent (send ckpt :motion-intent emark)))
        (ros::ros-info ";; ~S" (list intent emark))
        (case intent
          ((:absolute :relative)
           (return-from find-motion-checkpoint
             (let ((prev-ckpt (nth (1+ i) checkpoint-list)))
               (values ckpt intent (if prev-ckpt (send prev-ckpt :get-angle-vector))))))
          ((:displacement-absolute :displacement-relative)
           nil)
          (t
           (error value-error "Unknown motion intent: ~A" intent)))))
    nil))
