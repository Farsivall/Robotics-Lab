oboenv2) teaching@teaching-garbonzo:~/Downloads/Robotics-Lab-main(2)/Robotics-Lab-main$ python /home/teaching/Downloads/Robotics-Lab-main/mycobot_robot_control/src/mycobot_robot_control/mycobot_robot_control/pick_flow.py
pybullet build time: Oct 21 2025 10:07:17
[INFO] [1783612039.931942720] [mycobot_ik_client]: start ...
[INFO] [1783612039.932228683] [mycobot_ik_client]: share directory /home/teaching/mycobot_client/install/mycobot_client_2/share/mycobot_client_2
argv[0]=
file path ext:  /home/teaching/mycobot_client/install/mycobot_client_2/share/mycobot_client_2
b3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frameb3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
robot_baseb3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frameb3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
g_baseb3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
No inertial data for link, using mass=1, localinertiadiagonal = 1,1,1, identity local inertial frameb3Warning[examples/Importers/ImportURDFDemo/BulletUrdfImporter.cpp,126]:
gripper[INFO] [1783612040.170362396] [mycobot_ik_client]: Joint info simulator:
(0, b'robot_base_to_g_base', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'g_base', (0.0, 0.0, 0.0), (0.0, 0.0, 0.003), (0.0, 0.0, 0.0, 1.0), -1)
(1, b'g_base_to_joint1', 4, -1, -1, 0, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint1', (0.0, 0.0, 0.0), (0.0, 0.0, 0.026), (0.0, 0.0, 0.0, 1.0), 0)
(2, b'joint2_to_joint1', 0, 7, 6, 1, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint2', (0.0, 0.0, 1.0), (0.0, 0.0, 0.13956), (0.0, 0.0, 0.0, 1.0), 1)
(3, b'joint3_to_joint2', 0, 8, 7, 1, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint3', (0.0, 0.0, 1.0), (0.0, 0.0, 0.02948), (0.5000018366025517, 0.49999999999662686, -0.49999999999662686, -0.49999816339744835), 2)
(4, b'joint4_to_joint3', 0, 9, 8, 1, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint4', (0.0, 0.0, 1.0), (-0.1104, 0.0, -0.01628), (0.0, 0.0, 0.0, 1.0), 3)
(5, b'joint5_to_joint4', 0, 10, 9, 1, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint5', (0.0, 0.0, 1.0), (-0.096, 0.0, 0.049339999999999995), (0.0, 0.0, 0.7071080798594737, 0.7071054825112363), 4)
(6, b'joint6_to_joint5', 0, 11, 10, 1, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint6', (0.0, 0.0, 1.0), (0.0, -0.07318, 0.01678), (0.49999999999662686, -0.49999999999662686, 0.5000018366025517, -0.49999816339744835), 5)
(7, b'joint6output_to_joint6', 0, 12, 11, 1, 0.0, 0.0, -3.14, 3.14159, 1000.0, 0.0, b'joint6_flange', (0.0, 0.0, 1.0), (0.0, 0.0456, 0.019), (0.7071080798594737, 0.0, 0.0, 0.7071054825112363), 6)
(8, b'joint6output_to_gripper', 4, -1, -1, 0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, b'gripper', (0.0, 0.0, 0.0), (0.0, 0.0, 0.10600000000000001), (0.0, 0.0, 0.2588196364430847, 0.9659256678396477), 7)
[INFO] [1783612040.170641344] [mycobot_ik_client]: (null)
[INFO] [1783612040.170795392] [mycobot_ik_client]: Link info simulator:
(0.0, 0.5, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(1.0, 0.5, (1.0, 1.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.2, 0.5, (5e-05, 5e-05, 5e-05), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.2, 0.5, (5e-05, 5e-05, 5e-05), (0.0, 0.0, -0.03048), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.2, 0.5, (5e-05, 5e-05, 5e-05), (0.0, 0.0, 0.01628), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.2, 0.5, (5e-05, 5e-05, 5e-05), (0.0, 0.0, 0.01528), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.2, 0.5, (5e-05, 5e-05, 5e-05), (0.0, 0.0, -0.01678), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.2, 0.5, (5e-05, 5e-05, 5e-05), (0.0, 0.0, -0.019), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(0.05, 0.5, (2e-05, 2e-05, 2e-05), (0.0, 0.0, -0.006), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
(1.0, 0.5, (1.0, 1.0, 1.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), 0.0, 0.0, 0.0, -1.0, -1.0, 2, 0.001)
[INFO] [1783612040.171003105] [mycobot_ik_client]: (null)
[INFO] [1783612040.171146618] [mycobot_ik_client]: Link info pinocchio:
universe =   m = 0.2
  c =     0     0 0.029
  I =
5e-05     0     0
    0 5e-05     0
    0     0 5e-05
joint2_to_joint1 =   m = 0.2
  c =        0        0 -0.03048
  I =
5e-05     0     0
    0 5e-05     0
    0     0 5e-05
joint3_to_joint2 =   m = 0.2
  c =       0       0 0.01628
  I =
5e-05     0     0
    0 5e-05     0
    0     0 5e-05
joint4_to_joint3 =   m = 0.2
  c =       0       0 0.01528
  I =
5e-05     0     0
    0 5e-05     0
    0     0 5e-05
joint5_to_joint4 =   m = 0.2
  c =        0        0 -0.01678
  I =
5e-05     0     0
    0 5e-05     0
    0     0 5e-05
joint6_to_joint5 =   m = 0.2
  c =      0      0 -0.019
  I =
5e-05     0     0
    0 5e-05     0
    0     0 5e-05
joint6output_to_joint6 =   m = 0.05
  c =      0      0 -0.006
  I =
2e-05     0     0
    0 2e-05     0
    0     0 2e-05
[INFO] [1783612040.171506509] [mycobot_ik_client]: (null)
== home ==
home: already there
== open ==

== approach ==
waiting for /block_position from vision...
^CTraceback (most recent call last):
  File "/home/teaching/Downloads/Robotics-Lab-main/mycobot_robot_control/src/mycobot_robot_control/mycobot_robot_control/pick_flow.py", line 182, in <module>
    if not fn():
           ^^^^
  File "/home/teaching/Downloads/Robotics-Lab-main/mycobot_robot_control/src/mycobot_robot_control/mycobot_robot_control/pick_flow.py", line 108, in approach_pick
    px, py = wait_for_vision()
             ^^^^^^^^^^^^^^^^^
  File "/home/teaching/Downloads/Robotics-Lab-main/mycobot_robot_control/src/mycobot_robot_control/mycobot_robot_control/pick_flow.py", line 98, in wait_for_vision
    rclpy.spin_once(node, timeout_sec=0.1)
  File "/home/teaching/miniforge3/envs/roboenv2/lib/python3.12/site-packages/rclpy/_init_.py", line 208, in spin_once
    executor.spin_once(timeout_sec=timeout_sec)
  File "/home/teaching/miniforge3/envs/roboenv2/lib/python3.12/site-packages/rclpy/executors.py", line 776, in spin_once
    self._spin_once_impl(timeout_sec)
  File "/home/teaching/miniforge3/envs/roboenv2/lib/python3.12/site-packages/rclpy/executors.py", line 765, in _spin_once_impl
    handler, entity, node = self.wait_for_ready_callbacks(timeout_sec=timeout_sec)
                            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/teaching/miniforge3/envs/roboenv2/lib/python3.12/site-packages/rclpy/executors.py", line 748, in wait_for_ready_callbacks
    return next(self._cb_iter)
           ^^^^^^^^^^^^^^^^^^^
  File "/home/teaching/miniforge3/envs/roboenv2/lib/python3.12/site-packages/rclpy/executors.py", line 645, in _wait_for_ready_callbacks
    wait_set.wait(timeout_nsec)