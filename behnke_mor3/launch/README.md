# Readme file zur 3. MOR Übung


## Aufgabe 1: Kartographieren

Die Karte wurde mittels des ROS-Packages GMapping gemappt und unter behnke_map gespeichert.

Beim kartographieren wurde die Karte auf 5x5m begrenzt, da das Array im Suchalgorithmus sonst zu riesig wäre.

Das Launchfile _behnke_A1.launch_ startet die Gazebo Welt mit deaktivierter GUI sowie die Karte und den AMCL Lokalisierungsalgorithmus. Dies wird alles in Rviz visualisiert mit dem config-file _cfg/behnke_rviz_config_A1.rviz_.

**Usage:**
1. ```roslaunch behnke behnke_A1.launch```


## Aufgabe 2: Bestehende ROS Pfadplanung

Zur bestehenden ROS Pfadplanung wird move_base des turtlebot3_navigation Packages verwendet. 

Dies wird im Launch-file _behnke_A2.launch_ mit den dazugehörigen Parametern gestartet. 

Zusätzlich dazu wird das Ziel aufs Topic _/move_base_simple/Goal_ mittels des Befehls rostopic pub gepublisht.

Um die Visualisierung und Navigation zu starten muss zuvor das Launch-file für Aufgabe 1 ausgeführt werden.

**Usage:**
1. ```roslaunch behnke behnke_A1.launch```
2. ```roslaunch behnke behnke_A2.launch```



## Aufgabe 3.1: Eigene Pfadplanung

Für die eigene Pfadplanung muss ein Suchalgorithmus implementiert werden. In der Vorlesung wurden durchgemacht:
- Breitensuche
- Tiefensuche
- A* - Algorithmus

Zum Programmablauf -> siehe Kommentare im Code drinnen.

### Implementierter Algorithmus: Breitensuche

Ich habe mich als Suchalgorithmus für eine Breitensuche entschieden. Dieser durchsucht von der aktuellen Zelle immer erst alle nachbarn ab bevor er zum nächsten Knotenpunkt springt.
Der Vorteil der Breitensuche ist, dass er sich nicht in einer Endlosschleife verläuft und falls vorhanden auf jedenfall eine Lösung findet.
Nachteil ist, dass die Laufzeit im schlechtesten Fall lange dauert, da er erst alle Zellen absuchen muss um in der letzten Zelle das Ziel zu finden.
Dieses Problem besteht allerdings bei unserem Use case nicht, da die Karte nicht riesig ist.
Das besondere an der Breitenuche ist, dass die gefundene Lösung immer den kürzesten Weg zum Ziel hat.

Da die Pfadplanung den Pfad direkt an der Wand entlang berechnet, ohne die Breite des Turtlebots zu berücksichtigen, wurden beim Einlesen der Karte die Obstacles um 5 Zellen erweitert.


## Aufgabe 3.2: Eigene Pfadregelung

Zur Pfadregelung wurde die Node von MOR-Übung 2 zum Thema linearer Regelung erweitert, dass ein Pfad über das Topic eingelesen wird und die Regelung anschließend dem Pfad nachfährt.

Um die Visualisierung und eigenimplementierte Navigation zu starten muss zuvor das Launch-file für Aufgabe 1 ausgeführt werden.

**Usage:**
1. ```roslaunch behnke behnke_A1.launch```
2. ```roslaunch behnke behnke_A3.launch```

## Aufgabe 4: Visualisierung

Zur Visualisierung wurde das Config-file _behnke_rviz_config_A4.rviz_ erstellt. Dieses zeigt beide errechneten Pfade an und kann mittels einstellen des Parameter "PP_Mode" entscheiden nach welchem Pfad geregelt wird.

- PP_Mode = 1 bedeutet eigenimplementierte Nodes
- PP_Mode = 0 bedeutet bestehende ROS Nodes

**Usage:**
- Zum starten der Regelung nach der eigenimplementierten Pfadplanung:

  ```roslaunch behnke behnke_A4.launch PP_Mode:=1```

- Zum Starten der Regelung nach der ROS-Pfadplanung:

  ```roslaunch behnke behnke_A4.launch PP_Mode:=0```

Copyright - Fynn Behnke - mr18b070