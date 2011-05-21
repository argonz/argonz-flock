
;; gives inputs from the enviroment
(ns flock.sensor
  (:refer-clojure)
  (:refer msl)
  (:refer msl.seq-numerical)
  (:refer msl.math-funcs)
  (:refer msl.weight-lib))

;; sensor
;; s: b x bs x e -> {id [0 1]} or some other representation..
(defn sensor-union-f [& sensor-fs]
  (fn [b bs e]
   (reduce merge (map (fn [f] (f b bs e)) sensor-fs))))


;; sensor map
;; dir0 the lower coordinates of the direction of the sensor
;; dir1 the high
(defn angle->dir [angle]
  [(Math/cos angle) (Math/sin angle)])		;y x coordinates
(defn equidistial-angle-pairs [n]
  (let [r (/ (* 2 Math/PI) n)]
    (map (fn [i] [(* r i) (* r (inc i))]) (range n))))

;; x0 - y  x1 - x
(defn dir->10angle [dir]		;angle from the [1 0] direction
  (let [[x0 x1] dir
	a0 (Math/acos x0)
	a1 (Math/asin x1)]
    (if (< (Math/abs (- a0 a1)) 0.00001)
      a0
      (+ a0 Math/PI))))
(defn dirs->angle [dir0 dir1]
  (- (dir->10angle dir1) (dir->10angle dir0)))
(defn boid-between-angles? [b0 b e a0 a1]
  (let [a (dirs->angle (:dir b0) (flock.boid/direction b0 b e))]
    (<= a0 a a1)))
(defn boids-between-angles [b0 bs e a0 a1]
  (filter #(boid-between-angles? b0 % e a0 a1) bs))
(defn dist-nearest-boid-between-angles [b bs e a0 a1]
  (let [bs (boids-between-angles b bs e a0 a1)]
    (if (seq bs)
      (reduce min (map #(flock.boid/distance b % e) bs))
      nil)))

(defn between-angles-dist-sensor-f [id filter-f a0 a1 dist-f]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	(let [d (dist-nearest-boid-between-angles b fbs e a0 a1)]
	  (if d
	    {id (dist-f d)}
	    {id 0.0}))
	{id 0.0}))))
(defn between-angles-dist-sensors-f [ids filter-f dist-f]
  (let [angles (equidistial-angle-pairs (count ids))
	fs (map (fn [id [a0 a1]] (between-angles-dist-sensor-f id filter-f a0 a1 dist-f)) ids angles)]
    (fn [b bs e]
      (reduce merge (map (fn [f] (f b bs e)) fs)))))



;; creating the sensor function
(defn common-point-of-lines [a0 b0 a1 b1]
  (/ (- b0 b1) (- a1 a0)))
(defn common-point-of-nd-lines [a0s b0s a1s b1s]
  (map (fn [a0 b0 a1 b1] (common-point-of-lines a0 b0 a1 b1)) a0s b0s a1s b1s))
;; (defn bounds->bounding-lines [bounds]
;;   (let [n (count bounds)]
;;     (reduce concat (map (fn [i] 
;; 			  [[(replace-index (repeat n 1.0) i 0.0 
;; 			    (replace-index (repeat n 0.0) i (first (nth bounds i)))]
;; 			   (replace-index (repeat n 0.0) i (second (nth bounds i)))])
;; 			(range n)))))
;;   (let [dims (count bounds)]
;;     (for 
;; (defn common-point-of-dir-pos-and-line [dir pos 
     
;; (defn crossing-point-of-lines [c00 c01 x00 x01 c10 c11 x10 x11]
;;   ())

;; (defn boundary-between-angles-dist-sensor-f [ids filter-f dist-f]
;;   )

 

;; maybe use sensor-description

;; type-skeys [:typ [key .. ] ..]
;; type-akeys [:typ [key .. ] ..]
;; (defn boid-skeys-econst->between-angles-dist-sensors-f [typ skeys exp-const]
;;   (flock.sensor/between-angles-dist-sensors-f skeys 
;; 					      (flock.rules/filter-key-equal-f :typ typ)
;; 					      (flock.rules/dist-minus-exp-f exp-const)))
;; (defn sensor-params->sensor-union-f [& sensor-boid-skeys-params]
;;   (apply flock.sensor/sensor-union-f  
;; 	 (map (fn [sensor-typ boid-typ skeys params]
;; 		(cond (= sensor-typ :between-angles-dist) 
;; 		      (apply boid-skeys-econst->between-angles-dist-sensors-f boid-typ skeys params)))
;; 	      sensor-boid-skeys-params)))



;; actuator
(defn dir-from-between-angles-from-+1-1-actuators-f [actuator-field ids w]
  (let [angles (equidistial-angle-pairs (count ids))]
    (fn [b bs e]
      (let [ab (dir->10angle (:dir b))
	    dirs (map (fn [[a0 a1]] (angle->dir (+ (/ (+ a0 a1) 2.0) ab))) angles)
	    ws (map (fn [i] (get (actuator-field b) i)) ids)
	    ws (map (fn [w] (/ (+ 1 w) 2.0)) ws)] ;because of +1 -1
	(if (every? zero? ws)
	  [(:dir b) 0]
	  (do
;;	    (println ws)
	    [(seq-unitize (wavg-seqs dirs ws)) w]))))))
	    

;; (defn dir-from-actuator [actuator-data-f id0 id1 w]
;;   (fn [b bs e]
;;     [(seq-unitize [(get (actuator-data-f b) id0) (get (actuator-data-f b) id1)]) w]))
;; (defn dir-from-actuator [actuator-data-f id0 id1 w]
;;   (fn [b bs e]
;;     (let [dir [(get (actuator-data-f b) id0) (get (actuator-data-f b) id1)]
;; 	  dir (seq-unitize (if (= dir [0.0 0.0])
;; 			     [0.5 0.5] 
;; 			     dir))]
;;       [dir w])))

	



