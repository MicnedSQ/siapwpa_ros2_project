## Cel projektu 
Celem projektu było zaprojektowanie i wdrożenie algorytmu umożliwiającego autonomiczną jazdę 
po torze wyścigowym Sonoma Raceway, utrzymywania pojazdu na zadanym torze, w symulacji ROS2 + Gazebo
Zespół skupił się na zastosowaniu klasycznych metod sterowania (regulator PID).

## Założenia projektu
 - Samodzielne poruszanie się pojazdu po wirtualnym torze wyścigowym
 - Sterowanie - reulator PID reagujący na błędy w położeniu względem jezdni toru
 - Stała prędkość jazdy samochodu. Sterowanie samym kierunkiem (skrętem kół)

## Opis struktury projektu 

#### .devcontainer 
Zawiera konfiguracje kontenera z dependencjami potrzebnymi do uruchomienia projektu

#### centroids_msg
Definicja wiadomości dla pakietu ROS 2

#### simple_example 
Zaweira foldery:
 - #### launch
   example.launch,py - Uruchomienie śtodowiska testowego i inicjalizacja węzłów
 - #### models
   Model pojazdu 
 - #### simple_example
    - ControlerNode.py - implementuje regulator PID dla sterowania, obliczający błąd położenia względem środka toru i wyliczający sterowanie PID.

   - ProcessImageNode.py  - analiza obrazu z kamery i wykrywanie linii na drodze

 - #### test 
   line_detection_tests.py - testowanie wykrywania linii w określonym obszarze obrazu
 - #### worlds
   Pliki .sdf (Simulation Description Format) opisujace środowisko testowe oraz pojazd

#### setup.py
Skrypt konfiguracyjny dla pakietu ROS2

## Budowanie oraz sourcowanie projektu oraz sourcowanie projektu:

```
cd ~/ros2_ws
colcon build
source install/setub.bash
```

## Uruchamianie projektu:

W pierwszym terminalu, po wykonaniu sourcowania:

```
source install/setup.bash
```

należy uruchomić launch file:

```
ros2 launch simple_example example.launch.py
```

Następnie należy otworzyć dwa terminale i wykonać sourcowanie:

```
source install/setup.bash
```

W pierwszym terminalu należy uruchomić Node'a `process_image` wpisując następującą komendę:

```
ros2 run simple_example process_image
```

W drugim terminalu należy uruchomić Node'a `controller_node` wpisując następującą komendę:

```
ros2 run simple_example controller_node
```

Na sam koniec, w Gazebo należy nacisnąć ikonkę o nazwie `Run the simulation`: 
![RunTheSimulation](images/RunTheSimulation.png)