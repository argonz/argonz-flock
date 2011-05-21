;; BOUNDARY HANDLING
(ns flock.env.boundary
  (:refer-clojure)
  (:refer clojure.contrib.math)
  (:refer msl)
  (:refer msl.seq-numerical)) 
(defn make-boundary-handling [pos-bound-f displacement-f center-of-mass-f]
  {:pos-bound-f pos-bound-f
   :displacement-f displacement-f
   :center-of-mass-f center-of-mass-f
   :distance-f (fn [pos0 pos1] (seq-magnitude (displacement-f pos0 pos1)))
   :direction-f (fn [pos0 pos1] (seq-unitize (displacement-f pos0 pos1)))})

;; wall boundary
(defn walled-pos-bound-f [bounds]
  (fn [pos]
    (map (fn [x [min max]] 
	   (cond (< x min) min
		 (> x max) max
		 true x))
	 pos bounds)))  
(defn walled-bh [bounds]
  (make-boundary-handling (walled-pos-bound-f bounds) seq-displacement seqs-avg-seq))

;; NOT WORKING - reflective boundary - it should modify the direction of the boid..
(defn reflective-pos-bound-f [bounds]
  (fn [pos]
    (map (fn [x [min max]] 
	   (cond (< x min) (- min x)
		 (> x max) (- max x)
		 true x))
	 pos bounds)))
(defn reflective-bh [bounds]
  (make-boundary-handling (reflective-pos-bound-f bounds) seq-displacement seqs-avg-seq))

;; periodic boundary 
(defn periodic-pos-bound-f [bounds]
  (fn [pos]
    (map (fn [x [min max]] 
	   (cond (< x min) (+ x max)
		 (> x max) (- x max)
		 true x))
	 pos bounds)))
(defn periodic-displacement-f [bounds]
  (let [widths (map (fn [[w0 w1]] (- w1 w0)) bounds)]
    (fn [pos0 pos1]
      (map (fn [p0 p1 w]
	     (if (> (abs (- p1 p0)) (/ w 2.0))
	       (if (< p0 p1)
		 (- p1 p0 w)
		 (+ (- p1 p0) w))
	       (- p1 p0))) 
	   pos0 pos1 widths))))  

;; center of mass of a periodic enviroment? - project it to circle! :) 
;; http://www.cs.drexel.edu/~david/Papers/Bai_JGT.pdf 
(defn center-of-mass-periodic-1d [points min max]
  (let [w (- max min)
	rads (map (fn [p] (* (/ (- p min) w) (* 2 Math/PI))) points)
	cs  (map (fn [r] [(Math/sin r) (Math/cos r)]) rads)	
	c (seq-unitize (seqs-avg-seq cs))]
    (+ (* w (/ (Math/atan2 (nth c 0) (nth c 1)) (* Math/PI 2))) min)))
(defn center-of-mass-periodic-f [bounds]
  (fn [poss]
    (map (fn [ps [min max]] (center-of-mass-periodic-1d ps min max)) (ortolize poss) bounds)))
    
(defn periodic-bh [bounds]
  (make-boundary-handling (periodic-pos-bound-f bounds) 
			  (periodic-displacement-f bounds) 
			  (center-of-mass-periodic-f bounds)))

    
