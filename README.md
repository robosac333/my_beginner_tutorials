# my_beginner_tutorials
This repo is part is part of one of the excercises in ENPM700: Software Engineering in Robotics to create a simple Publisher Subscriber Node configuration

## Step 1: Create and navigate to your workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

## Step 2: Clone the repository
git clone https://github.com/robosac333/my_beginner_tutorials.git

## Step 3: Build the workspace
Build for proper clang-tidy search
```sh
cd ~/ros2_ws
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

## Step 4: Source the setup file
source install/setup.bash

## Instructions to run nodes:
To start the publisher node, use:
```sh
ros2 run beginner_tutorials talker
```

To start the subscriber node, open a new terminal, source the workspace, and use:
```sh
source ~/ros2_ws/install/setup.bash
ros2 run beginner_tutorials listener
```
To start the nodes using different logging level checks use the relevant arguments such as -debug or -warn. Example:
```sh
ros2 run beginner_tutorials talker --ros-args --log-level debug
```

## Service implementation
To trigger the service and change the logging message by the talker node use:
```sh
ros2 service call /change_message std_srvs/srv/Trigger
```
After using the service, the published string while be changed as follows:


You may run the service to provide a different msg

## Launching nodes
To launch both the nodes together and publish to a different frequency, you may use the following command:
```sh
ros2 launch beginner_tutorials talker_listener.launch.py frequency:=5.0
```

## TF transform broadcast
To check the changes in the tf-broadcast displaying transformation changes

```sh
ros2 run tf2_ros tf2_echo world talk
```

To create a pdf file for the ongoing rqt frames
```sh
ros2 run tf2_tools view_frames
```

## Record the bag file
In order to collect the messages published in the talker node inside a rosbag, run the talker node in a terminal.
In the other terminal run the following command:

```sh
ros2 launch beginner_tutorials bag_record.launch.py
```
The bag will collect messages and info coming from the running topics for 15 seconds and then close by saving the bag in the results folder

```sh
# Enable recording
ros2 launch beginner_tutorials bag_record.launch.py record_bag:=True

# Disable recording
ros2 launch beginner_tutorials bag_record.launch.py record_bag:=False
```

## Inspect the bag file

You might replace the path with the complete path to the bag file. This will provide the information about the info inside the rosbag
```sh
ros2 bag info src/my_beginner_tutorials/results/ros2_assg_3_imgs/rosbag2_2024_11_14-23_05_31
```

## Run the bag file

To check the messages coming from the bag, run the listener node in one window and in another run 

```sh
ros2 bag play src/my_beginner_tutorials/results/ros2_assg_3_imgs/rosbag2_2024_11_14-23_05_31
```

## Colcon-test

To test the integration unit tests run the following command after building from source

```sh
colcon test  --return-code-on-test-failure --event-handlers console_cohesion+ --packages-select beginner_tutorials
```

## cpp-lint
To check if the code base conforms to cpp-lint style
```sh
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order $(find . -name *.cpp | grep -v "/build/")
```

### Dependencies

- Ubuntu 22.04 (If running locally)
- ROS2 Humble
- Git
- C++17
- catch-ros2
