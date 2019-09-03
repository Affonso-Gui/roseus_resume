(ros::load-ros-manifest "action_interrupt")

;; define callback
(defmacro generate-sigint-handler-with-controller-interrupt (robot-interface service-name)
  `(compile
     (defun sigint-handler-with-controller-interrupt (c)
       (let ((status (ros::service-call
                      ,service-name
                      (instance action_interrupt::InterruptControllerRequest :init))))
         (send ,robot-interface :interrupt-angle-vector)
         (sigint-handler c)
         (send ,robot-interface :resume-angle-vector status)))))

;; install callback
(defmacro install-interruption-handler (robot-interface &optional (namespace "fullbody_controller"))
  (let ((service-name (format nil "~A/interrupt" namespace)))
    `(progn
       ;; set interruption signals
       (unix:system ,(format nil "rosrun action_interrupt stop_process ~A &" (unix:getpid)))

       ;; monitor controllers state
       (unless (ros::service-exists ,service-name)
         (unix:system
          (with-output-to-string (s)
            (format s "rosrun action_interrupt monitor_action")
            (format s " --namespace ~A" ,namespace)
            (dolist (ca (send ,robot-interface :get-val 'controller-actions))
              (format s " -c ~A" (send ca :name)))
            (format s " &"))))

       ;; generate & install interruption handler
       (generate-sigint-handler-with-controller-interrupt ,robot-interface ,service-name)
       (install-handler unix::sigint-received 'sigint-handler-with-controller-interrupt))))

(defmethod robot-interface
 (:interrupt-angle-vector ()
  (send self :cancel-angle-vector))
 (:resume-angle-vector (status &key (scale 1.0) (min-time 1.0))
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
             (send goal :trajectory :points (list (last-point))))

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

       
