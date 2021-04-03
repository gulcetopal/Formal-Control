## 1 Vehicle

1. roslaunch multihusky_gazebo traffic.launch 
2. roslaunch formal_control action_executor.launch
3. rosrun formal_control path_generator.py
4. rosrun formal_control driver_v3.py

## Multiple Vehicles
1. roslaunch multihusky_gazebo multicar_traffic.launch 
2. roslaunch formal_control action_executor.launch
3. rosrun formal_control path_generator.py
4. roslaunch formal_control drivers.launch
