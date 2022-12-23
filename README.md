# franka_example_controllers  
This repository adds some features based on franka_ros.  
- controller  
  - cartesian_admittance_controller_pose -- can not use  
  - cartesian_admittance_controller_torque -- can not use  
  - cartesian_admittance_controller_velocity -- can use  

- application  
  - DualArmMimic  
    ```sh
    # 回归原点
    roslaunch franka_example_controllers move_to_start.launch arm_id:=panda_1  
    roslaunch franka_example_controllers move_to_start.launch arm_id:=panda_2
    # 回到初始位置（并打开夹爪）
    roslaunch franka_example_controllers dual_arm_move_to_initial.launch arm_id:=panda_1
    roslaunch franka_example_controllers dual_arm_move_to_initial.launch arm_id:=panda_2
    # 启动策略  
    roslaunch franka_example_controllers dual_arm.launch
    ```