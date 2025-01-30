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