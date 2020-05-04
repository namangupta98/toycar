# ToyCar

## Author
 - Naman Gupta
 - Umang Rastogi
 - Aman Virmani

## Dependencies

 - ROS (Kinetic and above)
 - Ubuntu 16.04 and above

## Run Instructions

 Navigate to your <ROS_workspace>/src and type
 ```
 git clone https://github.com/namangupta98/toycar
 ```
 Build the package,
 ```
 cd ~/<ROS_workspace>
 catkin_make
 ```
 Now, run the controller package by typing,
 ```
 source devel/setup.bash
 roslaunch toycar controller.launch
 ```