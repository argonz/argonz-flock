(ns flock.test
  (:refer-clojure)
  (:refer msl.weight-lib)
  (:refer msl.seq-numerical)
  (:refer flock.boid)
  (:refer flock.2dgui)
  (:refer flock.rules)
  (:import java.awt.Color))

(defn random-boid [e]
  {:pos (flock.env/rand-pos e)
   :dir (flock.env/rand-dir e)
   :speed 1.5
   :size 0.5
   :update-f (flock.rules/united-update-rule  
	      :dir (flock.rules/dir-union-f 
		    (flock.rules/dir-relate-center-of-mass :away  
							   (flock.rules/filter-in-radius-f 2)
							   (flock.rules/dist-const-f 0.5))
		    (flock.rules/dir-relate-center-of-mass :toward
							   (flock.rules/filter-in-radius-f 50)
							   (flock.rules/dist-const-f 0.25))
		    (flock.rules/dir-relate-align-others (flock.rules/filter-in-radius-f 50)
							 0.2)
		    (flock.rules/dir-momentum 0.5))
	      ;; :speed (flock.rules/speed-union-f 
	      ;; 	    (flock.rules/speed-relate-center-of-mass (flock.rules/filter-in-radius-f 50)
	      ;; 						     (flock.rules/dist-linear-f 0.1)
	      ;; 						     (flock.rules/dist-const-f 0.05))
	      ;; 	    (flock.rules/speed-momentum 1.0))
	      :pos (flock.rules/pos-upd-f))
   :color (. Color GREEN)
   :boid-draw (flock.2dgui.boiddraw/rect-draw-f)})
   ;; :boid-draw (flock.2dgui.boiddraw/dir-line-draw-f)})

(defn demo-simple-flock []
  (let [bounds [[0 200] [0 180]]
	e {:bounds bounds
	   :pixel-per-meter 4
	   :update-f flock.env/e-update-boids 
	   :color (. Color BLACK)}
	boids (for [i (range 2)] (random-boid e))]
    (flock.2dgui/main (merge (flock.env.boundary/periodic-bh bounds)
			     (assoc e :boids boids)))))



