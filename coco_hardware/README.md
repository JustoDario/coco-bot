# COCO_HARDWARE

## Overview:
This packages defines all hardware components necesary for coco to be controlled via ros2_control, it includes:

* pca9865_hardware_interface: This hardware_interface receives positions(angles) that it will comunicate via I2C to the pca9865 in order to move the servos.