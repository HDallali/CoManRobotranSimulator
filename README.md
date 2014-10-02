CoManRobotranSimulator
======================

This is an open source simulator developed for the CoMan humanoid robot as part of WALK-MAN European project.

Requirements:

Robotran should be installed from www.robotran.be/downloads

In order to use the simulator:

1- Go to your MBProjects folder of Robotran

2- git clone https://github.com/HDallali/CoManRobotranSimulator.git

3- Go to the ../CoManRobotranSimulator/Standalone and create a build folder

4- Run ccmake ../ (set the required flags on)

5- In terminal: export YARP_ROBOT_NAME=CoMan

6- In terminal: export YARP_DATA_DIRS=(MBProjectsPaths)/MBProjects/CoManRobotranSimulator/Standalone/build/share/robotran


Regarding any problems please contact houman.dallali@iit.it and post your problems to http://www.robotran.be/forum/index .
