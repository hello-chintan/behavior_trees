# This is a project to experiment with task scheduling using the behavior tree library BehaviorTree.CPP

This project is compatible with version 4.X of [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP).

# Setup Instructions

## Install ROS 2 Humble
This project has a dependency on ROS 2 as it uses the colcon build system. Follow instructions for [installing ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

## Install BehaviorTree.CPP library
Begin by [creating a ROS 2 workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html):
```bash
mkdir -p ~/ament_ws/src
```

clone the BehaviorTree.CPP library along with this package using the following commands:
```bash
cd ~/ament_ws/src
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
git clone https://github.com/hello-chintan/behavior_trees.git
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

OUTPUT:
```bash
Starting execution of tasks!
Executing task A
Finished task A
Executing task B
Finished task B
Executing task C
Finished task C
Executing task D
Finished task D
Executing task E
Finished task E
Task execution ended successfully!
```

The code defines seven nodes (Start, End, and tasks named A to E) or actions that can be rearranged as necessary using Groot2 to achieve task dependency and scheduling. By default, tasks are arranged sequentially and are executed as such. However, they can be scheduled in different orders by editing the my_tree.xml file.

The tasks A to E all support asynchronous execution, which means that they are able to execute in parallel. While executing the tasks are in the RUNNING state and do not block the ticking mechanism of behavior tree.

Additionally, tasks A to E are able to share parameters between eachother using the parameter blackboard built into the BehaviorTree.CPP library. This can be used to modify the behavior of task execution.

Groot2 provides the visualization and logging capabilities. Behavior Trees being a composable and modular framework, all tasks in this exercise are reusable. Below is an image of how the visualization looks in Groot2.

