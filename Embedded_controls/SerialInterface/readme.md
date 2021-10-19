# BUG: Serial Interface Dokumentation

Auf das Main Interface kann auch über eine serielle Schnittstelle zugegriffen werden, um den Roboter zum Beispiel von einem PC aus zu steuern und die Sensoren auszulesen.
How to use:
- Benutze ``void initSchnittstelle()`` um serielle Verbindung, die Motoren und Sensoren zu initialisieren.
- Benutze `` void waitForSerialCommandAndExecuteAndSendStatus()`` in einem Loop, um Befehle von einer externen Quelle zu empfangen und auszuführen.
	Die Methode erwartet entweder den String:
	“init”, wodurch alle Sensoren und die Motoren neu initialisiert werden,
	oder einen mit Raute beginnenden String welcher Zielpositionen in Grad (0-180) von beliebig vielen Motoren enthält:
	``#1 90;#3 60;#4 70;....``, wobei die Motoren mit Hilfe des Main Interfaces angesteuert werden.
	Im Anschluss antwortet die Schnittstelle der externen Quelle mit Sensordaten in Form des Strings:
	``!<UltraschallDistanz>;<InfraRotGradanzahl>;<XRotation>;<YRotation>;<ZRotation>§``

Da der Arduino Nano standardmäßig nur ein seriellen Port besitzt, empfängt der Servo Controller den gleichen String.
Dieser reagiert jedoch nur auf mit “#” oder “Q” beginnende Strings.

Als externe Quelle wurde ein Pythonskript (Schnittstelle.py) mit der Bibliothek “pyserial” auf einem PC verwendet, welcher per USB mit dem Arduino verbunden ist.
Die Schnittstelle lässt sich mit:
```python
Schnittstelle(portname=<Portname>, number_of_motors=<Anzahl motoren>)
```
initialisieren.

Dafür wird der entsprechende Port geöffnet und ein paar sekunden gewartet, da der Arduino ein paar Sekunden benötigt um bereit zu sein.
Die Aktionen werden aus einem csv-file eingelesen und mit der Funktion ``ActionsToAngle`` in Grad umgerechnet. 
Anschließend werden die Positionen manuell an der Längsachse gespiegelt und Offsets addiert um die gleiche Ausgangssituation wie in der Simulation zu haben.
Mit der Funktion ``go_for_angles(angles)`` werden die Zielpositionen in der Form:
``#1 90;#3 60;#4 70;....``
an die Schnittstelle des Arduinos gesendet. 
Dessen Antwort (beginnend mit “!”) wird mit Hilfe der Funktion “readStatus()” entgegen genommen und ausgegeben. 
Ein Zyklus dauert über zwei Sekunden. Um eine höhere Geschwindigkeit zu erreichen kann man im Main Interface Fake Daten verwenden, anstatt die Sensoren auszulesen.
Durch den Verzicht auf die Sensordaten können alle 0,5 Sekunden ein neuer Befehl von der externen Quelle an die Schnittstelle gesendet werden.	