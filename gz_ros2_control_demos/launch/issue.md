warning reported


ros2 launch gz_ros2_control_demos ackermann_drive_example.launch.py

```
[gazebo-2] [INFO] [1744326415.243063141] [controller_manager]: Activating controllers: [ joint_state_broadcaster ]
[gazebo-2] Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (0,0): 0. Value should be positive and greater than zero.
[gazebo-2] Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (1,1): 0. Value should be positive and greater than zero.
[gazebo-2] Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (2,2): 0. Value should be positive and greater than zero.
[gazebo-2] Warning [Inertia.cpp:149] [Inertia::setMoment] Passing in an invalid moment of inertia matrix. Results might not by physically accurate or meaningful.
[gazebo-2] Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (0,0): 0. Value should be positive and greater than zero.
[gazebo-2] Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (1,1): 0. Value should be positive and greater than zero.
[gazebo-2] Warning [Inertia.cpp:233] [Inertia::verifyMoment] Invalid entry for (2,2): 0. Value should be positive and greater than zero.
[gazebo-2] Warning [Inertia.cpp:149] [Inertia::setMoment] Passing in an invalid moment of inertia matrix. Results might not by physically accurate or meaningful.
[gazebo-2] [WARN] [1744326415.702138926] [gz_ros_control]:  Desired controller update period (0.1 s) is slower than the gazebo simulation period (0.001 s).
```


https://en.wikipedia.org/wiki/List_of_moments_of_inertia