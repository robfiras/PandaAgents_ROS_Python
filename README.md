# PandaAgents_ROS_Python
This is a simple ROS Python interface meant to be used for real-world evaluation of reinforcement learning agents trained in [RLBench](https://github.com/stepjam/RLBench). This is build on the 
[ros_panda_controller](https://github.com/robfiras/ros_panda_controller), which is why the latter needs to be installed first.

## Installation
Install requirements
```bash
pip install -r requirements.txt
```

Install the package
```bash
pip install .
```


## Usage
Run the [ros_panda_controller](https://github.com/robfiras/ros_panda_controller) with the ip of your robot
```bash
roslaunch ros_panda_agent panda_controller.launch robot_ip:=your_robot_ip

```
Now you can use the interface in Python.

[Here](example/test_api.py) is simple example

```Python
import numpy as np
import rospy 
from ros_panda_interface import Robot

robot = Robot(home_gripper=True, width=0.0)

# open gripper and move to RLBench's default starting position
robot.open_gripper()
robot.move_to_home()

frequency = 20
episode_length = 40
rate = rospy.Rate(frequency)

print("Starting motion ...")
for i in range(episode_length):

    # get observation for your agent
    joint_positions = robot.get_arm_joint_positions()
    joint_velocities = robot.get_arm_joint_velocities()
    joint_torques = robot.get_arm_joint_torques()
    ee_position = robot.get_ee_position()
    gripper_finger_positions = robot.get_gripper_finger_positions()
    obs = np.concatenate([joint_positions, joint_velocities, joint_torques, ee_position, gripper_finger_positions])

    # let your fancy rl agent predict some actions based on the observations
    # actions = your_rl_agent.predict(obs)

    # here only some exemplary actions
    arm_actions = np.array([0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
    gripper_action = np.array([1.0])
    actions = np.concatenate([arm_actions, gripper_action])
    
    # send action
    robot.send_action(actions, disable_gripper=True)

    rate.sleep()

# stop in 1 sec
print("Stopping ...")
robot.stop(sec=1)

# grasp object
print("Grasping ...")
arm_actions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
gripper_action = np.array([0.0])
actions = np.concatenate([arm_actions, gripper_action])
robot.send_action(actions, disable_gripper=False)
print("Grasped!" if robot.check_if_object_grasped() else "Not Grasped...")

# sleep for 5 sec
rospy.sleep(2.0)

# open gripper and move to home position
robot.open_gripper()
robot.move_to_home()

print("Finished.")
```

## Limitations
Until now, this interface only supports joint velocities as the arm action type (c.f., [RLBench](https://github.com/stepjam/RLBench)). 