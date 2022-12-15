# franka_example_controllers  
This repository adds some features based on franka_ros.  
- controller  
  - cartesian_admittance_controller_pose -- can not use  
  - cartesian_admittance_controller_torque -- can not use  
  - cartesian_admittance_controller_velocity -- can use  

- application  
  - DualArmMimic
    - dual_arm.launch  
    - single_arm_velocity.launch
    - dual_arm_move_to_start.launch  
  
  - easy position control  
    - move_group_control.launch  
  
  - identify end-effector parameters  
    - collect.py  
    - identify.cpp  