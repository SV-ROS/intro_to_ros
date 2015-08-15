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

  The robot is built from a Raspberry PI II SBC and a Neato BV80 robot vacume base.
  Follow the instructions from Ubiquity Robotics for building a ROS image on the PI.
  https://github.com/UbiquityRobotics/ubiquity-misc
  Ensure you install the robot_state_publisher and robot_model packages from source else they will crash.
  
  
  git clone this repo to the catkin_ws/src folder on the PI
  do a catkin_make on the workspace
  
  Connect the BV80 to the PI via USB cable to the port inside the BV80 Dust Bin.
  
  Launch the ros packages from:
    roslaunch bv80bot_node bv80bot.launch
    
  you can set an argument in the /neato_node/launch/bringup-all.launch file to controll mapping vs nav
     ...
     <!--
        Set do_map to "true" to run the robot with gmapping to create a map.
        Set to "false" to run the robot with amcl to navigate within the map.
        
        save the map while it is running in gmapping using:
           rosrun map_server map_saver map:=/gmapping/map -f <your map name>
           
        update the tags in neato_2dnav/launch/move_base.launch to point to your map
        <node name="map_server" pkg="map_server" type="map_server" args="$(find neato_2dnav)/maps/<your map name>.yaml"/>
        
     -->
     <arg name="do_map" default="false" />
     ...
    
    
    You should launch rviz on a workstation to view and controll the robot.
    
    The launch file will  launch the xbox360 joystick telop package from the turtlebot packages, update the bv80bot.launch file to change you controller settings.
    
  
  
  
