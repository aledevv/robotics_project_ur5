<p align='center'>
    <h1 align="center">FOUNDAMENTAL OF ROBOTICS PROJECT </h1>
    <p align="center">
    University of Trento A.Y. 2022/2023
    </p>
    <p align='center'>
    Developed by:<br>
    DAlessandro De Vidi <br>
    Daniele Marisa <br>
    Giulia Modenese <br>
    Sofia Zandonà
    </p>   
</p>

## PROJECT DESCRIPTION
The objective of the project is to use the manipulator able to do autonomus pick-and-place operations.
Through the zed-camera, able to detect the classes and position of every block, the robot have to pick the object and place in the corrisponding potion.
## FOLDER STRUCTURE

## INSTALLATION
The project has been developed and tested on Ubuntu 20.04 with ROS Noetic, also we used the [locosim](https://github.com/mfocchi/locosim) repository for the ur5 simulation. The installation of the project is the following:
1) Clone the [locosim repository](https://github.com/mfocchi/locosim) and follow the respective instructions to install it
2) Clone this [repository](https://github.com/aledevv/robotics_project_ur5) in the `ros_ws/src` folder of the catkin workspace
3) Install the vision dependencies with the following command:
- Install YOLOv5 dependencies
   
```BASH
cd ~
git clone https://github.com/ultralytics/yolov5.git
cd yolov5
pip3 install -r requirements.txt
```
- Intall the other dependencies
```BASH
pip install torchvision==0.13.0
```
4) Compile the project with the following command:
```BASH
cd ~/ros_ws
catkin_make install
source install/setup.bash
```

## HOW TO RUN THE PROJECT
### SETUP
Inside  ``~/ros_ws/src/locosim/robot_control/base_controllers//params.py`` go to the line 46 and set:
```
'gripper_sim': True,
```

### RUN



