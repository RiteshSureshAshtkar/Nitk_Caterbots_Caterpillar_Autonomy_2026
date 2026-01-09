# Nitk Caterbots Caterpillar Autonomy 2026 Code
## i)Follow below steps to create and clone this repo
```bash
mkdir -p Caterpillar_ws/src
cd Caterpillar_ws
cd src
git clone
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

### Verify with
```bash
ros2 topic list
```
It should show
If it does not kindly disconnect all usb devices from RPI plug in the ESP32 and try again.If it still does not work please contact me
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
