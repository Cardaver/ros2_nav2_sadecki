Paczka to wielokaty.
Typek kazał zrobić to na topicu /goal_pose.

Skrótowy opis kodu:
- tworzenie publishera dla zadanej liczby boków i długości boku
- inicjalizacja zmiennych
- scan_callback - skanuje teren i wykrywa kolizje - jak robot będzie w zasięgu min_distance od przeszkody, to ma się zatrzymać
- odometry_callback - ma dać znak jak robot dojedzie do wierzchołka figury, aby rzucić kolejny topic z kolejnym wierzchołkiem
- update - rzucanie topic, żeby robot jeździł po wielokącie, którego środek jest w (0, 0):
    - uruchamiamy zegar
    - przemieszczamy robota poza punkt (0,0) - ma przejechać dystans r
    - liczymy mu orientację wyrażoną w kwaternionach z kątów eulera - tutaj kąt yaw
    - potem ma jeździć po kolejnych wierzchołkach aż liczba wierzchołków będzie się zgadzać

W razie pytań, pisać na DSC lub messenger do Sadeckiego.

Uruchamianie programu (według nas):

1) 4 terminale: gazebo, cartographer, nav2, program.
2) na każdym:
    - export TURTLEBOT3_MODEL=waffle
    - source install/local_setup.bash

Gazebo:
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Nav2:
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=~/turtlebot3_ws/src/install/turtlebot3_navigation2/share/turtlebot3_navigation2/map/map.yaml

Cartographer:
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

Program:
ros2 run wielokaty pub_wiel

W razie czego find . -name "map.yaml" jak Nav2 nie wykryje mapy.
