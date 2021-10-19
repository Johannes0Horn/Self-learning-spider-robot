## Main Interface 
Das Main Interface stellt eine logische Zwischenschicht dar, mit der auf den Roboter zugegriffen werden soll. Es integriert alle Sensoren sowie die Motorsteuerung und bietet ein simples interface zur Steuerung des Roboters.

### Funktionen
* ``void main_init()``
Initialisert und kalibriert die Motoren sowie alle Sensoren.
* ``struct Status main_step(float positions[18])``
Bewegt alle Motoren in die übergebenen Positionen, ließt anschließend Sensordaten aus und gibt diese als ``Status`` struct zurück. Das auslesen der Sensordaten verursacht eine kurze Verzögerung zwischen den steps. Vor dem Aufruf dieser Funktion muss ``main_init()`` ausgeführt werden.
* ``void main_step_without_sensors(float positions[18]``
Bewegt alle Motoren in die übergebenen Positionen. Ohne das auslesen der Sensordaten können steps bzw. actions schneller ausgeführt werden. Vor dem Aufruf dieser Funktion muss ``main_init()`` ausgeführt werden.

## Serielle Schnittstelle
Der Roboter erhält die auszuführenden Aktionen von einem Host Rechner.
Die serielle Kommunikation zwischen einem Host Rechner und dem Roboter erfolgt über ``schnittstelle.h`` auf dem Mikrocontroller und ``Schnittstelle.py`` auf dem Host Rechner.

### Funktionen
* ``void initSchnittstelle()``
Initialisiert die serielle Kommunikation.
* ``void waitForSerialCommandAndExecuteAndSendStatus()``
Wartet auf ein seriellen Befehl vom Host. Sobald dieser eingeht wird er ausgeführt und der aktuelle Status des Roboters über die serielle Schnittstelle an den Host zurückgeliefert. Vor dem Aufruf dieser Funktion muss ``initSchnittstelle()`` ausgeführt werden.

## Nutzung

### Stromversorgung des Roboters

Der Arduino Nano benötigt standardmäßig **5 Volt** Versorgungsspannung, der Servo Controller **4,8 - 6 Volt**. Für die Stromversorgung wird an den Servo Controller am besten ein externes Netzteil angeschlossen. Alternativ ist auch ein Akkubetrieb möglich. Der Arduino Nano wird über das Servo Controller Board mit spannung versorgt. In den Spitzen benötigen die Motoren **10 Ampere**.

### Flashen des Mikrocontrollers 
Ist der atmega328p des Roboters mittels **USB** an den Rechner angeschlossen, erfolgt das flashen mithilfe des Makefiles.
Über den Befehl
```
make unicorn
```
werden alle Files gebaut und auf den Controller geflasht.

### Steuerung mittels Host PC

Mithilfe eines geeigneten Main Files kann eine Bewegungsabfolge direkt auf den Mikrocontroller des Roboters geflasht werden. Um den Roboter zu trainieren oder gelernte Actions zu übertragen ist es jedoch sinnvoller, über die serielle Schnittstelle zu kommunizieren.

#### Mikrocontroller:

Das Main File muss sowohl ``main_interface.h`` als auch ``schnittselle.h`` importieren.
Um auf Befehle vom Host zu warten kann die folgende main Funktion ausgeführt werden:
```c
int main(void){
    initSchnittstelle();
    while (true){
        waitForSerialCommandAndExecuteAndSendStatus();
        _delay_ms(1000);
    }
    return 0;
}
```

#### Host PC

In ``Schnittstelle.py`` ist die serielle Kommunikation mit dem Mikrocontroller des Roboters implementiert. Das File enthält alle notwendigen Funktionen sowie eine Beispielhafte Nutzung, lediglich der USB Port muss ggf. angepasst werden.
Die actions aus dem vom Netzwerk generierten csv werden ausgelesen und an den Roboter übertragen. Dieser liefert nach jedem step seinen aktuellen Status zurück.
