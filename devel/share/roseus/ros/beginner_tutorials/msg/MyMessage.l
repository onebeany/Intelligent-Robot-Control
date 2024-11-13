;; Auto-generated. Do not edit!


(when (boundp 'beginner_tutorials::MyMessage)
  (if (not (find-package "BEGINNER_TUTORIALS"))
    (make-package "BEGINNER_TUTORIALS"))
  (shadow 'MyMessage (find-package "BEGINNER_TUTORIALS")))
(unless (find-package "BEGINNER_TUTORIALS::MYMESSAGE")
  (make-package "BEGINNER_TUTORIALS::MYMESSAGE"))

(in-package "ROS")
;;//! \htmlinclude MyMessage.msg.html


(defclass beginner_tutorials::MyMessage
  :super ros::object
  :slots (_time _nums ))

(defmethod beginner_tutorials::MyMessage
  (:init
   (&key
    ((:time __time) "")
    ((:nums __nums) (make-array 0 :initial-element 0 :element-type :integer))
    )
   (send-super :init)
   (setq _time (string __time))
   (setq _nums __nums)
   self)
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:nums
   (&optional __nums)
   (if __nums (setq _nums __nums)) _nums)
  (:serialization-length
   ()
   (+
    ;; string _time
    4 (length _time)
    ;; int32[] _nums
    (* 4    (length _nums)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _time
       (write-long (length _time) s) (princ _time s)
     ;; int32[] _nums
     (write-long (length _nums) s)
     (dotimes (i (length _nums))
       (write-long (elt _nums i) s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _time
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _time (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32[] _nums
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _nums (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _nums i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;;
   self)
  )

(setf (get beginner_tutorials::MyMessage :md5sum-) "c0c0cdfb81e8e5ff82904cd630ef5b88")
(setf (get beginner_tutorials::MyMessage :datatype-) "beginner_tutorials/MyMessage")
(setf (get beginner_tutorials::MyMessage :definition-)
      "string time
int32[] nums

")



(provide :beginner_tutorials/MyMessage "c0c0cdfb81e8e5ff82904cd630ef5b88")


