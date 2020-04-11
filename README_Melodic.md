Here are some quick notes on running under Ubuntu 18.04 and ROS-Melodic. 
I was able to get it working with some minor headaches.

After a full install of Ubuntu 18.04 and ros-melodic-ddesktop-full

  add the default user to the dialout group
  apt install or check that openssh-server is installed
  apt install ros-melodic-turtlebot* (may be overkill)  and ros-melodic-yocs*
  so far I havent been able to locate the turtlebot-teleop-joy node in melodic, 
      so I don't have a joystick working.
    
  Assuming you've built a catkin_ws,  git clone this repository and catkin_make
  
  I was using an older Acer small laptop originally supplied with a Turtlebot2.
  Make sure your dev/tty is USB0 or ACM0 and that it is properly noted in the bv80bot drivers,
  by default it is ttyACM0, but some computers point to ttyUSB0.
  
  If you are attached to the Botvac USB port, launching the bv80bot_node base_only.launch should start the Lidar spinning.
  
  There will be some error messages about not finding the joystick.
