INSTRUCTIONS TO START WORKING ON THIS/TEST FK NODE:

Once you've followed the Github steps that the TA provided on setting up a workspace with necessary packages:
- Remove previous .git folder
- Go to src/final_proj and clone in this github
- Run `colcon build --packages-select final_proj --symlink-inst all`
- open a bunch of terminals \(tmux, manually, or run some stuff in background\) that are in the original workspace level, **not** the src


<h1>RUNNING THE FORWARD KINEMATICS:</h1>
- Plug in to robot with USB connector \(and plug robot power source into wall\)
- **Each terminal you use should be at the workspace, and start with you sourcing ros** \(hopefully in your `.bashrc`\) **and then running** `source install/setup.bash`

- First terminal \(or in bg\) to start the robot controller: 
run `ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py`. This will power the motors and let you look at the topics.

- Second terminal: save this one for positional control. 
You will be running `ros2 run final_proj basic_robot_control`. 
This will tell the arm to set its joints to the positions specified in `pos_example.py` on line 23.

- Third terminal: This will be the forward kinematics publisher. 
Run `ros2 run final_proj fwd_kin`. This should constantly output the joint positions to the terminal. This is expected.

- Fourth terminal: This will be the forward kinematics subscriber \(and print the information to the terminal\). 
Run `ros2 topic echo /ee_pose`. This should constantly output the end effector pose \(Point and Quaternion representation\).

<h1>RUNNING INVERSE KINEMATICS FOR PART 1 PROJECT</h1>
- Build Workspace
`cd ~/Desktop/rbe500_project/RBE500`
`colcon build --packages-select final_proj`
`source install/setup.bash`

- Terminal 1:
`ros2 run final_proj ik_service --ros-args -p x:=200.0 -p y:=100.0 -p z:=150.0`

- Terminal 2:
`ros2 service call /compute_ik std_srvs/srv/Trigger`
