# intro_to_ros
Repository of packages and info for the SV-ROS Intro To ROS training series

This repo will contain the software and documents for the SV-ROS 2015 Intro to Ros training series.

The series involes a set of talks presenting a general introduction to ROS and building ROS Robots.

Reference Robot designs are provided as a guide to getting started.

The first of these is a robot built on the Neato BV80 base and using a Raspberry PI II SBC.

The ROS packages and drivers for this robot can be found in the bv80bot folder(s).

The driver is based on a modified Neato Driver first created by Michael Ferguson.

--
bv80bot
-------

  The robot is built from a Raspberry PI II SBC and a Neato BV80 robot vacume base.
  
  Follow the instructions from Ubiquity Robotics for building a ROS image on the PI.
  https://github.com/UbiquityRobotics/ubiquity-misc
  
  <b>Ensure you install the robot_state_publisher and robot_model packages from source else they will crash.</b>
  
  Also install ALL of the following:
  ```
  sudo apt-get install ros-indigo-xacro ros-indigo-turtlebot-description ros-indigo-turtlebot-navigation ros-indigo-turtlebot-teleop ros-indigo-yocs-cmd-vel-mux ros-indigo-yocs-velocity-smoother
```
  
  Git clone this repo to the catkin_ws/src folder on the PI and you Laptop/PC.
  
  <b>Note: do this on both your PC/Laptop and the Raspberry PI.</b>, 
  
  you need copies of the files on both computers.
  
  ```
  cd ~/catkin_ws/src
  git clone https://github.com/SV-ROS/intro_to_ros.git
  ```
  do a catkin_make on the workspace (on both computers)
  
  ```
  roscd
  cd ..
  catkin_make
  ```
  
  Also make sure you have chrony installed on both the PC/Laptop and the PI:
  
  ```
  sudo apt-get install chrony
  ```
 
  There are a number of different way to launch the robot depending on where you want to run the packages.
  
  The robot can be run in two modes - mapping and navigation, you must first run the robot in mapping mode to create a   map to use later for navigation.
  
  
  Packages for mapping an navigation can be run either on the robot or the PC/Laptop, the following gives the launch     commands for each configuration:
  
  <b>Mapping Mode:</b>
  
    Launch the ros packages for mapping on the robot:
    
    on the Raspberry PI -- roslaunch bv80bot_node bv80bot_base_map.launch
    on the PC/Laptop    -- roslaunch bv80bot_node bv80bot_gui_only.launch
    
    
    Launch the ros packages for mapping on the PC/Laptop:
    
    on the Raspberry PI -- roslaunch bv80bot_node bv80bot_base_only.launch
    on the PC/Laptop    -- roslaunch bv80bot_node bv80bot_map_gui.launch
    
Drive around with the joystick/keyboard until you have a good enough map (see below for telop configuration).

Once you have crated a map you like you must save it before you stop running the nodes launched above.
You can save the map to the PI or the Laptop/PC, you should save it to the computer you intend to run the nav nodes on later.

So on the apropreate computer, (PC/Laptop or the PI) save the map as follows:
```
roscd neato_2dnav/maps
rosrun map_server map_saver
```
The map will be saved as two files in the .../neato_2dnav/maps folder, map.yaml, map.pgm


  <b>Navigation Mode:</b>
  
    Launch the ros packages for navigation on the robot (if you saved your map to the PI):
    
    on the Raspberry PI -- roslaunch bv80bot_node bv80bot_base_nav.launch
    on the PC/Laptop    -- roslaunch bv80bot_node bv80bot_gui_only.launch
    
    
    Launch the ros packages for navigation on the PC/Laptop (if you saved your map to the PC/Laptop):
    
    on the Raspberry PI -- roslaunch bv80bot_node bv80bot_base_only.launch
    on the PC/Laptop    -- roslaunch bv80bot_node bv80bot_nav_gui.launch
    
  You should now be able to set the robots pose in rviz and set nav goals for the robot to goto with the 2d nav goal button in rviz.
    
    
  <b>Teleop Configuration:</b>
  
  The launch files mentioned can be configured to use 1 of 4 teleop controllers:
  - ps3 joystick
  - xbox360 joystick
  - logitech joypad
  - keyboard

  There is an argument at the top of the bv80bot_base.launch file as shown below:

  ~/catkin_ws/src/intro_to_ros/bv80bot/bv80bot_node/launch/include/bv80bot_base.launch
  ```
  <!-- Change this to use a different joystick controller -->
  <!-- Set the default value for the 'teleop_controler' arg below to one of the folowing values to select the controler you are using to teleop the robot:
        ps3
        xbox360
        keyboard
        logitech
  -->
  <arg name="teleop_controler"   default="xbox360" />
  ```
  
  Change the value of the default argument shown above to one of the indicated choices to change your controler settings/configuration.
  
    
  
  
  
