#!/bin/bash
ssh 'franka@tueirsi-nc-008' 'pkill -f franka_control_node'
ssh 'franka@tueirsi-nc-008' '/home/franka/franka_ros/src/panda_moveit_control_only/launch/remoteEnv.bash'
