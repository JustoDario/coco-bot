# COCO_BRINGUP

## Overview:
This is the package responsible to launch every necesary node/controller for Coco to start functioning.It launchs:

    - robot_description
    - robot_state_publisher
    - ros2_control_node
    - joint_state_broadcaster
    - joint_state_controller
    - gait_planifier_node
    - rviz(optional)

When launched Coco awaits commands/interaction. 