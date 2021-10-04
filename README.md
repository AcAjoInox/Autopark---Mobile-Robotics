# Autopark---Mobile-Robotics
Progetto del corso Mobile Robotics aa2020/2021
DESCRIZIONE

I pacchetti pose e object_spawner servono allo spawn randomico in numero e in tipologia di modello dei veicoli nei 28 parcheggi
Nel pacchetto autopark si trova il file urdf del car-like robot ferrari.urdf
il controller utilizzato è il file python cmdvel2gazebo.py che riconduce il car-like robot ad un uniciclo
Nel file dist_obj.py viene implementato il sensing
Nel file trajectory_generation viene generata la traiettoria di parcheggio
nel file trajectory_tracking è lo script principale

PER FAR FUNZIONARE IL PROGRAMMA :

lanciare il comando "roslaunch autopark autopark_car.launch"
una volta spawnate tutte le auto lanciare "rosrun autopark trajectory_tracking.py"

N.B. : per poter inseguire la traiettoria quadrata rinominare i file trajectory_tracking_old.py e trajectory_generation_old.py
       in trajectory_tracking.py e trajectory_generation.py . La traiettoria viene correttamente generata tuttavia bisogna implementare  
       un controllore funzionante per garantire l'inseguimento.
