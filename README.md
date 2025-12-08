# LAB2 - Eater vs. Killer: Spawn–Forage–Pursuit (SFP) with RViz2 and Turtlesim+

1. **Clone the repository**
   ```bash
   cd
   git clone -b LAB2 https://github.com/phattanaratjeedjeen-sudo/FRA502-LAB-6810.git
    ```
2. **Build the workspace**
   ```bash
   cd ~/FRA502-LAB-6810
   colcon build
   source install/setup.bash
    ```
3. **Run nodes** (Open new terminal everytime before run node)

   ```bash
   ros2 run turtlesim_plus turtlesim_plus_node.py 
   ```
    ```bash
   ros2 run lab2 eater.py
   ```
    ```bash
   ros2 run lab2 killer.py
   ```
    ```bash
   ros2 run lab2 turtlesim_pose.py
   ```
    ```bash
   ros2 service call /spawn_turtle turtlesim/srv/Spawn "x: 1.0
   y: 1.0
   theta: 0.0
   name: 'turtle2'" 
    ```
    ```bash
   cd ~/FRA502-LAB-6810
   rviz2 -d src/lab2.rviz
   ```
