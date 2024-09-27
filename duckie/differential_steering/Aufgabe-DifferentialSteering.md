# Aufgabe Differential Steering

Der mobile Roboter Duckiebot soll in diesem Laborversuch so steuerbar sein, dass er ohne Probleme eine bestimmte Position im Raum anfahren kann. Dazu steuert er seine Räder an und verwendet als Sensor die Encoder der Räder. Diese geben an, um wie viele `Ticks` sich das Rad gedreht hat.

Wie ihr die Räder ansteuert, habt ihr im Unterricht kennengelernt. Schaut euch ansonsten nochmals die Python Datei `wheel_command_publisher.py` oder auch `breitenberg.py` an.

Die *Wheel Ticks* könnt ihr mit folgendem *Topic* auslesen:

    /ROBOT_NAME/left_wheel_encoder_node/tick
    /ROBOT_NAME/right_wheel_encoder_node/tick

Wie die *Message* `WheelEncoderStamped` definiert ist, seht ihr mit

    rosmsg  show duckietown_msgs/WheelEncoderStamped

oder auch [hier](https://github.com/duckietown/dt-ros-commons/blob/daffy/packages/duckietown_msgs/msg/WheelEncoderStamped.msg) im Quellcode.


Erstellt eure eigenen Python Scripts im selben `duckie_lab` Paket.


## Teilaufgabe 1: Die *Wheel Encoder* verstehen

a) Abonniert die *Wheel Ticks* und dreht von Hand die Räder. Welche Informationen erhaltet ihr? Und was bedeuten diese?

b) Schreibt ein Programm, welches den Duckiebot auf der Stelle drehen lässt, so dass er genau am selben Punkt wieder zu stehen kommt.


## Teilaufgabe 2: Wissen, wo der Duckiebot ist

Berechnet anhand der `WheelTicks` die Pose $P_{robot} = (x, y, \theta)$, wo sich der Roboter befindet, wenn er sich fortbewegt.


Ihr könnt die berechnete Pose in *Iviz* anzeigen. Schaut euch dazu diese [Anleitung](iviz.md) an.

Die Geometrie des Roboters wird schon von anderen ROS Knoten publiziert. Ihr könnt also den Roboter virtuell fortbewegen, indem ihr angibt, wie man vom Koordinatensystem `map` zum Koordinatensystem `ROBOT_NAME/base` kommt. Dazu könnt ihr die Funktion 

    publish_transform(x, y, theta)

verwenden, die ihr in der `differential_steering.py` findet.


## Teilaufgabe 3: Die Fahrt steuern

a) Definiert eine gewisse Startposition $S = (x=0, y=0, \theta=0)$ für euren Roboter und eine Zielposition $G = (x_{goal},y_{goal},\theta_{goal})$ mit Hilfe von Malerband, welches ihr auf den Boden kleben könnt.

b) Vermesst die Zielposition und steuert den Duckiebot so an, dass er theoretisch die Zielposition erreichen sollte.

c) Wie gross ist die Differenz zwischen Ist- und Sollposition? Was beeinflusst die Differenz?


## Teilaufgabe 3: Die Fahrt regeln

a) Benutzt nun die Informationen der *Wheel Encoder*, um die Fahrt zu regeln. Als Einstieg zum Thema Regler kann euch dieser [Wikipedia Artikel](https://de.wikipedia.org/wiki/Regler) dienen.

b) Fahrt mithilfe des Reglers die Zielposition aus Teilaufgabe 3 an und versucht zusätzlich, mehrere Zielpositionen hintereinander anzufahren.

c) Inwiefern verbessert sich das Verhalten? Was braucht es, um den Roboter noch präziser fahren zu lassen?



</br>
</br>

---
**Weiterführende Literatur**: Hertzberg et al. Mobile Roboter, Springer Verlag 