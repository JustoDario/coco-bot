# COCO_MOV_CONTROL

## Overview

This package includes:

* gait_planifier class: It turns geometry_msgs/msg/Twist to trajectory_msgs/msg/JointTrajectory for the JointTrajectoryController.This allows Coco to move via cmd_vel by  Nav2 , teleop_twist_key and such.

* emote_planifier class: It turns  coco_msgs/msg/Emote to the corresponding joints positions as trajectory_msgs/msg/JointTrajectory.
    Available emotes:
        - simple_dance
        - jump
        - stretch
        - handshake
