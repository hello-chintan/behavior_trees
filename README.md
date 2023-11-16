# This is a project to experiment with task scheduling using the behavior tree library BehaviorTree.CPP

This project is compatible with version 4.X of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).

# Setup Instructions

## Install ROS 2 Humble
This project has a dependency on ROS 2 as it uses the colcon build system. Follow instructions for [installing ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Install BehaviorTree.CPP library
Begin by [creating a ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html):
```bash
mkdir -p ~/ament_ws/src
cd ~/ament_ws/src
```

clone the BehaviorTree.CPP library along with this package using the following commands:
```bash
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
git clone ...
```

Build the workspace using colcon to ensure smooth progress:
```bash
cd ~/ament_ws
colcon build
```

Next, install Groot2, the BT GUI tool, to visualize, edit and log behavior trees. Follow instructions on the [Groot2 installation page](https://www.behaviortree.dev/groot/).

# Running the code
You are now ready to run the code. Assuming everything is setup correctly, execute the code in example_one.cpp as follows:
```bash
ros2 run behavior_trees btcpp_sample
```
