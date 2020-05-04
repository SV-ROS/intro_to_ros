# intro_to_ros
Repository of packages and info for the SV-ROS Intro To ROS training series

This repo will contain the software and documents for the SV-ROS 2015 Intro to Ros training series.

As of January, 2018, many updates have occurred, so we will be updating the documentation.

The series involves a set of talks presenting a general introduction to ROS and building ROS Robots.

Reference Robot designs are provided as a guide to getting started.

The first of these is a robot built on the Neato BV80 base and using a Raspberry PI III. The PI III has built in WiFi and Blue Tooth, but because of this the default names of ttyUSB0  needs to be changed to ttyACM0 in order to connect to the robot.

The ROS packages and drivers for this robot can be found in the bv80bot folder(s).

The driver is based on a modified Neato Driver first created by Michael Ferguson.


## Background info
The following link provides the basic command set for the BV80 over the USB serial port.

Also type HELP in a terminal connected to the port to get the built in command help info.

USB Serial API doc - https://tinyurl.com/Neato-Programmers-Manual

Other useful info on the Lidar

https://github.com/rohbotics/xv11hacking/tree/master/mainSpace

--
## Setingup the Robot/Rasperry PI
-------

### Initial SD Card Image

  The robot is built from a Raspberry PI III SBC and a Neato BV80 or later robot vacuum base.
  
  To start use a blank 16Gb or 32Gb SDcard and install the Ubuntu Image from Ubiquity robotics:
  
  Follow the instructions from Ubiquity Robotics for building a ROS image on the PI.
  https://downloads.ubiquityrobotics.com   This will allow you to place a ROS image on a 16Gb or 32Gb SD card which can 
  then directly boot the PI with ubuntu 16.04 and ROS Kinetic pre installed. Since this image is for another robot, 
  additional modifications to the PI's OS will be required to make a working robot as detailed below.
  
 ### Setup the Intro to ROS Packages 
 
After creating the SDCard insert it into the Rasperry PI and boot it - at this point its best if you have an HDMI monitor, mouse and keyboard attached or you can work via an SSH terminal session.
 
Initial login using:

     user -  ubuntu      
     password-  ubuntu
     
First disable the pre loaded ubiquity robotics ROS stacks that are auto started with the following command:

```sudo systemctl disable magni-base```

If you don't always want to login as the ubuntu user you may want to create a new user - just follow standard Ubuntu admin steps to do this.

Login as your prefered user.

Use the normal Ubuntu networking steps to connect to a suitable WiFi network.

Now update the installed image:

```
   sudo apt-get update
   sudo apt-get upgrade
```

Make sure ROS files are up to date:

  ```
  rosdep update
  ```

Create a new ROS catkin workspace under your home directory, then git clone the intro_to_ros repo to the catkin_ws/src folder.
  
  <b>Note: do this on both your PC/Laptop and the Raspberry PI.</b>, 
  
  you need copies of the files on both computers.
  
  ```
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws/src
  git clone https://github.com/SV-ROS/intro_to_ros.git
  ```
  Optional, but highly recommended:
  
  ```
   git clone https://github.com/pirobot/rbx1
   git clone https://github.com/vanadiumlabs/arbotix_ros
  ```

  
Also install the depedancies using the following command:


```
cd ~/catkin_ws

rosdep install --from-paths src --ignore-src -r -y
```

If you receive an error: 
```'Cannot locate rosdep definition for [yocs-velocity-smoother]'```
you will need to install it manualy

```sudo apt-get install ros-<distro>-yocs-velocity-smoother```

  
  do a catkin_make on the workspace (on both computers)
  
  ```
  cd ~/catkin_ws
  catkin_make
  ```
  
  Now you can update the .bashrc file in your home directory.

  open ~/.bashrc in vi or whatever text editor you prefer.

  Near the end find the line:
  
   ```
     source /opt/ros/<distro>/setup.bash
   ```

  comment out this line and replace it as shown:
 
```
  # source /opt/ros/<distro>/setup.bash
  source ~/catkin_ws/devel/setup.bash
```

Note: in the above change &lt;distro&gt; to kinetic or melodic depending on what platform you are using
  
  Save and close the editor

  now close and reopen a terminal or just source ~/.bashrc

  You should now be able to launch the ROS code as discussed below.




 ## Running the Robot
 
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

So on the appropriate computer, (PC/Laptop or the PI) save the map as follows:
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
  
    
  
  
  
