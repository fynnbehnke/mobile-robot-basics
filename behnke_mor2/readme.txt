Dies ist das package für die 2.Übung in MOR.

Startprozedur (Einzelprogramm):
- roslaunch behnke_mor2 start_sim.launch (startet die gazebo-simulation (gui daktiviert) und eine Rviz visualisierung)
- roslaunch behnke_mor2 move_to_goal.launch x:=... y:=... theta:=... (startet die Regelung um die übergebene Pose anzufahren. x,y,theta werden hierbei als floats übergeben)

Startprozedur (Stange umfahren):
- navigieren in den Ordner "launch"
- starten mit dem bash skript ./start_all.sh (fährt 4 Punkte um die Stange und dann zurück auf die Startposition 0,0,0)

Die Node stick_pub dient dazu die Stange aus Gazebo in Rviz visualisieren zu können. Sie publisht einen visualization Marker in Form eines Zyllinders an der Stelle der Stange.

Die Karte sowie das Launchfile myfile.launch wurden aus dem Angabeprojekt (baseproject) implementiert und die Pfade entsprechend angepasst.

mfg,
Fynn Behnke - mr18b070
