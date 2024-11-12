# Control Box ROS Bridge
This repository collects the necessary code to control a pneumatic-driven Continuum Soft Manipulators through a Control Box.

## Hardware & Software Setup
To include the bridge in your ROS project, you need this steps.

1. Select the Control Box for your project. In our lab, we have 3 control boxes, with different numbers of valves.
2. Connect the control box with your device by USB. 
*Be sure to have turned on the compressor and the power supply of the control box.*
3. Clone this bridge in your workspace typing in the terminal
```bash
cd ~/<your_ws_path>/src
git clone git@github.com:Elektron97/controlbox_bridge.git
```
4. The build it
```bash
cd ..
catkin_make
```
5. Update the [hardware parameters](config/hardware_params.yaml) file for your specific control box. Usually, you can measure them by **digital pressure gauge**.

## Usage
To launch the bridge node
```bash
roslaunch controlbox_bridge controlbox_bridge.launch
```
The node subscribe to the topic `/pressures` (`Float32MultiArray` msg type) and it will send by serial comm the command to the arduino inside the controlbox.

## Contribution & Bug Report
If you encounter some bugs or you would like to contribute, you can open a new branch and implement your improvements.
```bash
# Creating your branch
git checkout -b <your_branch_name>
```
Moreover, you can also open an issue in this GitHub repository to report bug or ask help.