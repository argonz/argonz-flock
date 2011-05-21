(ns flock) 

;; BOID
(ns flock.boid
  (:refer-clojure)
  (:refer msl)
  (:refer msl.seq-numerical)
  (:refer msl.weight-lib))

;; elementary boid
(defn init-boid [id pos dir speed size update-f]
  {:id id
   :pos pos
   :dir dir
   :speed speed		                ;in meter-per-second --- ugh.. I don't really want to work with this now? :)
   :size size				;in meter - it's only important for representing..
   :update-f update-f})			;update-f boid x {boids/self} x env -> new-boid

;; utility functions for boids
(defn id->boid [id bs]
  (find-first #(= (:id %) id) bs))
(defn typ->boids [typ bs]
  (filter #(= (:typ %) typ) bs))

(defn pos-bound [b e]
  ((:pos-bound-f e) (:pos b)))
(defn distance [b0 b1 e]
  ((:distance-f e) (:pos b0) (:pos b1)))
(defn displacement [b0 b1 e]
  ((:displacement-f e) (:pos b0) (:pos b1)))
(defn direction [b0 b1 e]
  ((:direction-f e) (:pos b0) (:pos b1)))
(defn center-of-mass [bs e]
  ((:center-of-mass-f e) (map :pos bs)))

;; (defn update-dir [boid env]
;;   (assoc boid :dir (seq-unitize (:dir boid))))
(defn in-radius [boid boids env r]
  (remove (fn [b] (> (distance b boid env) r)) boids))
(defn nearest [boid boids env]
  (reduce #(if (< (distance %1 boid env) (distance %2 boid env))
	     %1
	     %2)
	  boids))

;; FLOCKING ENVIROMENT

;; env
;; :pos-bound-f :pos-distance-f :pos-direction-f 
;; :boids
;; :color

;; metric datas - well everything is fine.. just the display should be tweaked a little..
;; :time-step - refresh the stuff in that second.. 
;; :time      - the time has elapsed
;; :pixel-per-meter   -  exchange ratio, speed etc is added in meter-per-second
;; :bounds - in meter

(ns flock.env
  (:refer-clojure)
  (:refer msl))

;; boid state update
(defn boid->upd [b e]
  ((:update-f b) b (remove #(identical? % b) (:boids e)) e))
(defn boids->upd [bs e]
  (map #(boid->upd % e) bs))
(defn e->upd-boids [e]
  (assoc e :boids (boids->upd (:boids e) e)))

;; boid sensor update
(defn boid->sens-data [b e]
  (if (:sensor-f b)
    ((:sensor-f b) b (remove #(identical? % b) (:boids e)) e)))
(defn boids->sens-data [bs e]
  (reduce merge (map #(boid->sens-data % e) bs)))
(defn e->boid-sens-data [e]
  (boids->sens-data (:boids e) e))

;; (defn boid->upd-sens [b e]
;;   (if (:update-sens-f b)
;;     ((:update-sens-f b) b (remove #(identical? % b) (:boids e)) e)
;;     b)) 
;; (defn boids->upd-sens [bs e]
;;   (map #(boid->upd-sens % e) bs))
;; (defn e->upd-sens [e]
;;   (assoc e :boids (boids->upd-sens (:boids e) e)))
;; (defn e->boid-sens-data [e]
;;   (reduce merge (map :sensor-data (:boids e))))

;; boid actuator update
(defn boid->upd-acts [b act-d]
  (let [common-keys (clojure.set/intersection (set (keys (:actuator-data b))) (set (keys act-d)))]
    (assoc b :actuator-data (reduce (fn [h k] (assoc h k (get act-d k))) {} common-keys))))  
(defn boids->upd-acts [bs act-d]
  (map #(boid->upd-acts % act-d) bs))
(defn e->upd-acts [e act-d]
  (assoc e :boids (boids->upd-acts (:boids e) act-d)))
 
;; time update
(defn e->upd-time-with-time-step [e]
  (assoc e :time (+ (:time e) (:time-step e))))


;; utility functions
(defn bound-widths [e]
  (map (fn [[x0 x1]] (- x1 x0)) (:bounds e)))
(defn id->boid [id e]
  (flock.boid/id->boid id (:boids e)))
(defn typ->boids [typ e]
  (flock.boid/typ->boids typ (:boids e)))
(defn rand-pos [e]
  (msl.rand/rand-seq (:bounds e)))
(defn rand-dir [e]
  (msl.seq-numerical/seq-unitize (for [i (range (count (:bounds e)))] (rand 1.0))))
