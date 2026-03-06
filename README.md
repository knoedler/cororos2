# cororos2
ROS2 port for some robots

Background:
Contest is DARPA Triage Challenge
Here is a 360 view of one of the courses from last year (viewer discretion - has simulated injuries, use the mouse to scroll around)  You can see some of another teams robot running.  https://youtu.be/eNnbdQN-Kik?list=PL6wMum5UsYvYlCNFhd9Y7MMURvPKbh07J&t=369

Robot Type 1 - Cornelius / Julius
Initial Model - https://app.gazebosim.org/OpenRobotics/fuel/models/coro_hd2_sensor_config_1
Relevant Differences from Model - LIDAR is different model, D455 is about 1.5 meters up and tilted about 30 degrees down
Platform is SuperDroidRobots HD2
Motor Controller is Roboclaw 2x60A controller
Ouster OS0-128 LIDAR
Memsense MS-IMU3025 IMU
u-blox ZED-F9P GPS in RTK mode
Intel D455 depth camera
Laptop for control (currently running Ubuntu 20)

ROS1 roboclaw driver is in repository, ROS2 driver appears available - Cornelius has motor feedback but is not using it, Julius does not the encoders connected
u-box ROS1 driver is in repository, ROS2 may be available
Memsense ROS1 driver is in repository, probably needs ported


Robot Type 2 - Allie / Ames
Initial Model - https://app.gazebosim.org/OpenRobotics/fuel/models/coro_allie_sensor_config_1
Relevant Differences from Model - LIDAR is different model, D455 is about 1.5 meters up and tilted 30 degrees down
Custom Platform
Motor Controller is revrobotics SPARK MAx - using RC PWM input
Ouster OS0-128 LIDAR
Memsense MS-IMU3025 IMU
u-blox ZED-F9P GPS in RTK mode
Intel D455 depth camera
Laptop for control (currently running Ubuntu 20)

Motor driver is just simple conversion from speed/turn to PWM - no feedback


Robot Type 3 - Joe / Jeanine / Kepler / Jack / Susan
Initial Model - https://app.gazebosim.org/OpenRobotics/fuel/models/CORO_JEANINE_SENSOR_CONFIG_1
Relevant Differences from Model - D455 is about 1.5 meters up and tilted 30 degrees down
Custom Platform using re-purposed hoverboard motors
Motor Controller is ODrive v3.6
Velodyne VLP-16 LIDAR
Memsense MS-IMU3025 IMU
u-blox ZED-F9P GPS in RTK mode
Intel D455 depth camera
Laptop for control (currently running Ubuntu 20)

Odrive ROS1 Motor driver is in repository - ROS2 probably available - all motors have feedback
Motors are a bit low on torque, so depending on surface it can be challenging to turn in place

