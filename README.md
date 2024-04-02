# OkayPlan:
[![okayplan_ros](https://github.com/XinJingHao/Images/blob/main/OkayPlan/okayplan_youtube.png)](https://www.youtube.com/watch?v=zufvYkhRW5w)
<div align=center>
  <img src="https://img.shields.io/badge/Python-blue" />
  <img src="https://img.shields.io/badge/ROS-ff69b4" />
  <img src="https://img.shields.io/badge/PathPlanning-blueviolet" />
</div>

## 1.Installation
Please change the following ```your_ws``` with your own ```ROS_workspace```

- **Step1:** clone okayplan_ros to your_ws
```cmd
cd ~/your_ws/src
git clone https://github.com/XinJingHao/okayplan_ros.git
```

The file structure should be like this:
```
~/your_ws
  ├── build
  ├── devel
  └── src
      ├── other-packages
      └── okayplan_ros
          ├── FML_Robot
          ├── include
          ...
          ├── CMakeLists.txt
          ├── package.xml
          └── requirements.txt
```

- **Step2:** compile your workspace
```cmd
cd ~/your_ws/src/okayplan_ros/scripts
chmod +x *.py
cd ~/your_ws
catkin_make
```
If the ros packages listed in the ```CMakeLists.txt``` haven't been installed, you need to install them manually by ```sudo apt install ros-noetic-<package>``` 

- **Step3:** create an anaconda environment
```cmd
conda create -n myenv python=3.8.3
```

- **Step4:** install python dependencies
```cmd
cd ~/your_ws/src/okayplan_ros
conda activate myenv
pip3 install -r requirements.txt
```

## 2.Runing
Please change the following ```your_ws``` with your own ```ROS_workspace```

- **Step1:** launch the ros simulation platform
```cmd
echo "source ~/your_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
roslaunch okayplan_ros okayplan_car5.launch
```


- **Step2:** run the OkayPlan algorithm
```cmd
# Open a new terminal
cd ~/your_ws/src/okayplan_ros/Planner_and_Env
conda activate myenv
python main.py
```

## 3.Dependencies
```c++
# System related:
  Ubuntu=20.04
  ROS=Noetic
  gazebo=11.11.0
  rviz=1.14.20
  python=3.8.3

# Python related:
# install with **pip3 install -r requirements.txt**
  torch>=2.2.0 # GPU version required!
  torchaudio>=2.2.0
  torchvision>=0.17.0
  matplotlib>=3.7.5
  numpy>=1.24.4
  rospkg>=1.5.0
  rospy>=1.16.0
  catkin_tools>=0.9.4

# ROS packages related:
# install with **sudo apt install ros-noetic-<package>**
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  move_base
  roscpp
  rospy
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  urdf
  xacro
```

## Citing this Project

To cite this repository in publications:

```bibtex
@misc{OkayPlan,
      title={OkayPlan: Obstacle Kinematics Augmented Dynamic Real-time Path Planning via Particle Swarm Optimization}, 
      author={Jinghao Xin and Jinwoo Kim and Shengjia Chu and Ning Li},
      year={2024},
      eprint={2401.05019},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

