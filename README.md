# autopark
project in mobile Robotics 20/21 

DESCRIZIONE 
- i pacchetti pose e object_spawner servono allo spawn randomico in numero e in tipologia di modello dei veicoli nei 28 parcheggi
- nel pacchetto autopark si trova il file urdf del car-like robot ferrari.urdf
- il controller utilizzato è il file python cmdvel2gazebo.py che riconduce il car-like robot ad un uniciclo
- Nel file dist_obj è implementato il sensing
- Nel file trajectory_generation viene generata la traiettoria di parcheggio
- Il file trajectory_tracking è lo script principale 

N.B. Per generare la traiettoria quadrata rinominare i file trajectory_generation_old.py e trajectory_tracking_old.py in trajectory_generation.py e 
     trajectory_tracking.py . La traietoria desiderata viene correttamente generata . L'inseguimento è da completare con la sintesi di un controllore 
     funzionante.

PER FAR FUNZIONARE IL PROGRAMMA :
- lanciare il comando "roslaunch autopark autopark_car.launch"
- una volta spawnate tutte le auto lanciare "rosrun autopark trjectory_tracking.py"
