# FRA502-LAB-6810
Phattanarat Jeedjeen 6810 (Wa)

# LAB3 - Eater vs. Killer: Spawn–Forage–Pursuit (SFP) with RViz2 and Turtlesim+

**Source** https://github.com/kittinook/ARCS2/tree/LAB3?tab=readme-ov-file


## Usage
1. **Clone the repository**
   ```bash
   cd ~
   git clone -b LAB3 https://github.com/phattanaratjeedjeen-sudo/FRA502-LAB-6810.git
    ```
2. **Build the workspace**
   ```bash
   cd ~/FRA502-LAB-6810
   colcon build
   source install/setup.bash
    ```

   ```bash
   echo "source ~/FRA502-LAB-6810/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
4. **Run nodes**

   ```bash
   ros2 launch lab3 lab3_bringup.launch.py
   ```

