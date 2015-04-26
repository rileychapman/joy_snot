# joy_snot
joystick control of snotbot multicopter

This is a ROS package for all the code written to bridge the gap between a joystick and a multirotor. Initially, this will be a bridge between the joy and roscopter nodes.

# Running
To run, launch roscore in the terminal, then type roslaunch joy_snot joy_snot.launch also from the terminal.

Additional launch files are hover_joy.launch, which is a simple launch-land autonomy code that can be initiated by the joystick (and overridden).

Joy_snot_multi.launch allows you to control several drone vehicles at once using a single joystick.  There is currently a command delay between drones (the actions are not simultaneaus, there is a 0.5second difference), so if you decide to use this script, use extreme caution when flying.

# Dependencies
In order to get the most from this package, download roscopter from https://github.com/epsilonorion/roscopter and follow this tutorial https://github.com/epsilonorion/roscopter/wiki

Be sure you can run your multirotor with roscopter by typing rosrun roscopter driver.py --device=/dev/ttyUSB0 --baudrate=57600.  Echo the /attitude node to determine if you have a connection.

To run a joystick, you will need to download joystick drivers: sudo apt-get install ros-hydro-joystick-drivers for your version of ROS.  You may need to change the permissions of your joystick to the 'dialout' group, in which case the following links may be useful to you http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick, http://linuxcommand.org/lts0070.php, http://askubuntu.com/questions/26179/what-do-the-groups-do-in-users-and-groups. 

# Hardware
This code was written for a Logitech Extreme 3D Pro Controller (button 3 Arms, button 4 Disarms).  You can do what you want!  This code is functional for ROS Hydro, Ubuntu 12.04.


