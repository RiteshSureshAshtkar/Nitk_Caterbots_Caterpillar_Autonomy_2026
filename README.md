# Nitk Caterbots Caterpillar Autonomy 2026 Code
## i)Follow below steps to create and clone this repo
```bash
mkdir -p Caterpillar_ws/src
cd Caterpillar_ws
cd src
git config --global http.version HTTP/1.1
git clone https://github.com/RiteshSureshAshtkar/Nitk_Caterbots_Caterpillar_Autonomy_2026.git
```
## ii)Connect ESP32 first(without plugging in the lidar)
### a) Open this whole repository on VS Code and install the PlatformIO platform plugin on VS Code by following [this](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
### b) Then open ONLY mcu_pio folder of this repo on PlatformIO and upload code to the ESP32
#### Alternatively you can use Arduino IDE to do the same thing but all the code from mcu_pio folder of this repo has to be formatted in Arduino format and then uploaded
## iii)Now we will build all the ROS2 packages
```bash
cd
cd Caterpillar_ws
colcon build
source install/setup.bash
```
## iv)Now we will be running blitz(Simplified packer and parser for serial communication between your device and ESP32 on rover)
```bash
ros2 launch blitz blitz.launch.py
```
This should launch up and show
[blitz_node.py-1] [INFO] [1768130641.357372987] [blitz_node]: [PUB] /encoder_raw
[blitz_node.py-1] [INFO] [1768130641.358857268] [blitz_node]: [PUB] /rover_limit_sw
[blitz_node.py-1] [INFO] [1768130641.360230568] [blitz_node]: [PUB] /mode_switch
[blitz_node.py-1] [INFO] [1768130641.361655590] [blitz_node]: [PUB] /bno
[blitz_node.py-1] [INFO] [1768130641.365965933] [blitz_node]: [SUB] /velocity
[blitz_node.py-1] [INFO] [1768130641.367565974] [blitz_node]: [SUB] /actuator_cmd
[blitz_node.py-1] [INFO] [1768130641.368279883] [blitz_node]: BlitzNode started on /dev/ttyUSB0 @ 115200

### Verify with below command in new terminal
```bash
ros2 topic list
```
It should show
/encoder_raw
/rover_limit_sw
/mode_switch
/bno
/velocity
/actuator_cmd

## In the same terminal run
```bash
ros2 topic echo /bno
```
This should show up values if it does not repeat step iv) from start

Make sure you are using a data cable and not a power only cable
If it does not still work then kindly disconnect all usb devices from RPI plug in only the ESP32 and try again.If it still does not work please contact me
## v)Now let us launch the file which will start up the SLAM_Mapping and map_auto_saver node
### Open new terminal and copy paste below commands
```bash
cd Caterpillar_ws
source install/setup.bash
ros2 launch rover manual.launch.py
```
## vi)Open rviz
```bash
ros2 run rviz2 rviz2
```
### Now select add and select by topic type and select /map
