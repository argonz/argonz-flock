
;; rules for updating boids
(ns flock.rules
  (:refer-clojure)
  (:refer msl)
  (:refer msl.seq-numerical)
  (:refer msl.weight-lib)
  (:refer flock))


;; b bs e -> filtered-bs
(defn filter-comb-f [& filters]
  (fn [b bs e]
    (reduce (fn [bs f] (f b bs e)) bs filters)))
(defn filter-nofiltering-f []
  (fn [b bs e]
    bs))
(defn filter-in-radius-f [r]
  (fn [b bs e]
    (flock.boid/in-radius b bs e r)))
(defn filter-key-equal-f [key val]
  (fn [b bs e]
    (filter #(= (key %) val) bs)))

;; dir -> weight functions
(defn dist-const-f [c]
  (fn [d] c))
(defn dist-linear-f
  ([c max] (fn [d]
	     (let [v (* c d)]
	       (if (< max v)
		 max
		 v))))
  ([c] (fn [d] (* c d))))

(defn minus-exp [x c]
  (Math/exp (* (- c) x))) 

;; [1 0] descending
(defn dist-minus-exp-f 
  ([c max] (fn [d] 
	     (* (Math/exp (* (- c) d)) max)))
  ([c] (dist-minus-exp-f c 1.0)))

;; [0 1] ascending
(defn dist-per-minus-exp-f
  ([c max] (fn [d]
	     (* (Math/exp (/ (- c) d)) max)))
  ([c] (dist-per-minus-exp-f c 1.0)))

;; [0 inf] ascending
(defn dist-exp-f
  ([c max] (fn [d] 
	     (let [v (* (Math/exp (* c d)) max)]
	       (if (< max v)
		 max
		 v))))
  ([c] (fn [d] (Math/exp (* c d)))))


;; this creates a function which will update the correct fields
;; rules return the new value
(defn united-update-rule [& field-rules]
  (let [frs (tupelize field-rules)]
    (fn [b bs e]
      (reduce (fn [nb [f r]] (assoc nb f (r b bs e))) ;the rules use the old values :O
	      b
	      frs))))

;; position rules
;; (defn update-pos [boid env]
;;   (let [npos (seq-+-seq (:pos boid) 
;; 			(seq-*-n (seq-unitize (:dir boid)) 
;; 				 (* (:time-step env) (:speed boid))))]
;;     (assoc boid :pos ((:pos-bound-f env) npos))))
(defn pos-upd-f []
  (fn [b bs e]
    (let [npos (seq-+-seq (:pos b) (seq-*-n (:dir b) (* (:time-step e) (:speed b))))]
      (flock.boid/pos-bound {:pos npos} e))))
    ;; (let [npos (seq-+-seq (:pos b) (seq-*-n (:dir b) (:speed b)))]
    ;;   (flock.boid/pos-bound {:pos npos} e))))

;; direction rules 
;; b X bs X e -> v w
(defn dir-union-f [& rules]
  (fn [b bs e]
    (let [vws (map (fn [r] (r b bs e)) rules)
	  [vs ws] (ortolize vws)]
      (seq-unitize (wavg-seqs vs ws))))) ;unitize is important!!1

(defn dir-momentum [w]
  (fn [b bs e]
    [(:dir b) w]))      
(defn dir-relate-nearest [dir filter-f dist-weight-f]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	(let [nb (flock.boid/nearest b fbs e)
	      w (dist-weight-f (flock.boid/distance b nb e))]
	  (cond (= :away dir) [(flock.boid/direction nb b e) w]
		(= :toward dir) [(flock.boid/direction b nb e) w]))
	[(:dir b) 0]))))
(defn dir-relate-center-of-mass [dir filter-f dist-weight-f]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	(let [c {:pos (flock.boid/center-of-mass bs e)}
	      w (dist-weight-f (flock.boid/distance b c e))]
	  (cond (= :away dir) [(flock.boid/direction c b e) w]
		(= :toward dir) [(flock.boid/direction b c e) w]))
	[(:dir b) 0]))))
(defn dir-relate-align-others [filter-f dist-weight-f]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	(let [nb (flock.boid/nearest b fbs e)]
	  [(seq-unitize (reduce #(seq-+-seq %1 %2) (map :dir fbs))) (dist-weight-f (flock.boid/distance b nb e))])
	[(:dir b) 0]))))


;; speed rules 
;; b X bs X e -> v w
(defn speed-union-f [& rules]
  (fn [b bs e]
    (let [vws (map #(% b bs e) rules)
	  [vs ws] (ortolize vws)]
      (wavg vs ws))))
(defn speed-const [v w]
  (fn [b bs e]
    [v w]))
(defn speed-momentum [w]
  (fn [b bs e]
    [(:speed b) w]))
(defn speed-relate-nearest [filter-f dist-new-speed-f dist-weight-f]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	(let [nb (flock.boid/nearest b fbs e)
	      nd (flock.boid/distance b nb e)]
	  [(dist-new-speed-f nd) (dist-weight-f nd)])
	[(:speed b) 0]))))
(defn speed-relate-center-of-mass [filter-f dist-new-speed-f dist-weight-f]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	(let [c {:pos (flock.boid/center-of-mass bs e)}
	      d (flock.boid/distance b c e)]
	  [(dist-new-speed-f d) (dist-weight-f d)])
	[(:speed b) 0]))))
(defn speed-align-others [filter-f w]
  (fn [b bs e]
    (let [fbs (filter-f b bs e)]
      (if (seq fbs)
	[(avg (map :speed bs)) 0]
	[(:speed b) 0]))))
(defn speed-from-actuator [actuator-data-f id max w]
  (fn [b bs e]
    [(* max (get (actuator-data-f e) id)) w]))




