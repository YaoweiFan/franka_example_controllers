# franka_example_controllers  
This repository adds some features based on franka_ros.  
- controller  
  - cartesian_admittance_controller_pose   
    位置可实现导纳控制，但方向还不行   
    现象：在末端施加一定力矩后就报错 --> 关节加速度不连续   
  - operational_space_controller   
    设置高增益后可以使用，但具有稳态误差  
  - cartesian_admittance_controller_velocity  
    可以使用  

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