;; Auto-generated. Do not edit!


(when (boundp 'rvo_ros::SetGoals)
  (if (not (find-package "RVO_ROS"))
    (make-package "RVO_ROS"))
  (shadow 'SetGoals (find-package "RVO_ROS")))
(unless (find-package "RVO_ROS::SETGOALS")
  (make-package "RVO_ROS::SETGOALS"))
(unless (find-package "RVO_ROS::SETGOALSREQUEST")
  (make-package "RVO_ROS::SETGOALSREQUEST"))
(unless (find-package "RVO_ROS::SETGOALSRESPONSE")
  (make-package "RVO_ROS::SETGOALSRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass rvo_ros::SetGoalsRequest
  :super ros::object
  :slots (_model _coordinates ))

(defmethod rvo_ros::SetGoalsRequest
  (:init
   (&key
    ((:model __model) "")
    ((:coordinates __coordinates) ())
    )
   (send-super :init)
   (setq _model (string __model))
   (setq _coordinates __coordinates)
   self)
  (:model
   (&optional __model)
   (if __model (setq _model __model)) _model)
  (:coordinates
   (&rest __coordinates)
   (if (keywordp (car __coordinates))
       (send* _coordinates __coordinates)
     (progn
       (if __coordinates (setq _coordinates (car __coordinates)))
       _coordinates)))
  (:serialization-length
   ()
   (+
    ;; string _model
    4 (length _model)
    ;; geometry_msgs/Point[] _coordinates
    (apply #'+ (send-all _coordinates :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _model
       (write-long (length _model) s) (princ _model s)
     ;; geometry_msgs/Point[] _coordinates
     (write-long (length _coordinates) s)
     (dolist (elem _coordinates)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _model
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _model (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Point[] _coordinates
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coordinates (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _coordinates)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(defclass rvo_ros::SetGoalsResponse
  :super ros::object
  :slots (_num_goal ))

(defmethod rvo_ros::SetGoalsResponse
  (:init
   (&key
    ((:num_goal __num_goal) 0)
    )
   (send-super :init)
   (setq _num_goal (round __num_goal))
   self)
  (:num_goal
   (&optional __num_goal)
   (if __num_goal (setq _num_goal __num_goal)) _num_goal)
  (:serialization-length
   ()
   (+
    ;; int64 _num_goal
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _num_goal
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _num_goal (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _num_goal) (= (length (_num_goal . bv)) 2)) ;; bignum
              (write-long (ash (elt (_num_goal . bv) 0) 0) s)
              (write-long (ash (elt (_num_goal . bv) 1) -1) s))
             ((and (class _num_goal) (= (length (_num_goal . bv)) 1)) ;; big1
              (write-long (elt (_num_goal . bv) 0) s)
              (write-long (if (>= _num_goal 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _num_goal s)(write-long (if (>= _num_goal 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _num_goal
#+(or :alpha :irix6 :x86_64)
      (setf _num_goal (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _num_goal (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass rvo_ros::SetGoals
  :super ros::object
  :slots ())

(setf (get rvo_ros::SetGoals :md5sum-) "269a0b03d7a509c517f6ff76c318c9eb")
(setf (get rvo_ros::SetGoals :datatype-) "rvo_ros/SetGoals")
(setf (get rvo_ros::SetGoals :request) rvo_ros::SetGoalsRequest)
(setf (get rvo_ros::SetGoals :response) rvo_ros::SetGoalsResponse)

(defmethod rvo_ros::SetGoalsRequest
  (:response () (instance rvo_ros::SetGoalsResponse :init)))

(setf (get rvo_ros::SetGoalsRequest :md5sum-) "269a0b03d7a509c517f6ff76c318c9eb")
(setf (get rvo_ros::SetGoalsRequest :datatype-) "rvo_ros/SetGoalsRequest")
(setf (get rvo_ros::SetGoalsRequest :definition-)
      "string model
geometry_msgs/Point[] coordinates

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
int64 num_goal



")

(setf (get rvo_ros::SetGoalsResponse :md5sum-) "269a0b03d7a509c517f6ff76c318c9eb")
(setf (get rvo_ros::SetGoalsResponse :datatype-) "rvo_ros/SetGoalsResponse")
(setf (get rvo_ros::SetGoalsResponse :definition-)
      "string model
geometry_msgs/Point[] coordinates

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z
---
int64 num_goal



")



(provide :rvo_ros/SetGoals "269a0b03d7a509c517f6ff76c318c9eb")


