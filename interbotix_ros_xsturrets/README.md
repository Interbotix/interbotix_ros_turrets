# InterbotiX X-Series Turret ROS Packages
![xsturret_banner](images/xsturret_banner.png)

## Overview
Welcome to the *interbotix_ros_xsturrets* sub-repo! This repository contains ROS packages meant to be used with the many [X-Series robotic turrets](https://www.trossenrobotics.com/c/robot-turrets.aspx)  sold by Trossen Robotics. Packages were tested on Ubuntu Linux 16.04, 18.04, and 20.04 using ROS Kinetic, Melodic, and Noetic respectively. Additionally, all ROS nodes were written using Python or C++. However, any programming language capable of sending ROS messages can be used to control the robots. To that effect, the core packages that make up this repo are as follows:
- **interbotix_xsturret_gazebo** - contains the config files necessary to launch a turret in Gazebo, including tuned PID gains for ros_control
- **interbotix_xsturret_control:** contains the motor configuration files and the 'root' launch file that is responsible for launching the turret
- **interbotix_xsturret_descriptions:** contains the meshes and URDFs (including accurate inertial models for the links) for all turret platforms

Finally, there is also an **examples** directory containing various demos of how the above mentioned core packages can be used. So what are you waiting for? Let's get started!

## IRROS Structure
Refer [here](https://github.com/Interbotix/interbotix_ros_core#code-structure) to get a general understanding of IRROS.
![xsturret_irros_structure](images/xsturret_irros_structure.png)

##### Hardware Layer
All X-Series turrets are made up of [X-Series Dynamixel servos](https://www.trossenrobotics.com/dynamixel-x-series-robot-servos). Each servo has two 3-pin JST ports that allows it to be daisy chained with other servos using 3-pin cables. The 'root' Dynamixel (i.e. the 'pan' motor) then connects to the [XM/XL motor power hub](https://www.trossenrobotics.com/3-pin-x-series-power-hub.aspx). Besides for providing 12V to the motors from the barrel jack, the hub also connects to the 3-pin JST port on the [U2D2](https://www.trossenrobotics.com/dynamixel-u2d2.aspx). This device acts as a communication interface between a computer (connected via microUSB cable) and the motors - converting USB/TTL signals back and forth.

On the right side of this layer, there are two sensors. While the base robot kit from Interbotix does not come with either a USB camera or joystick controller, there are some demos (ROS packages in the Application layer) that may use them.

##### Driver Layer
The ROS packages in this sub-repo build up from the *interbotix_xs_sdk* ROS wrapper found in the *interbotix_ros_core* repository. Reference the package there for implementation details. The *usb_cam* and *joy* packages are ROS wrappers around a USB camera and PS3/PS4 controller devices respectively.

##### Control Layer
The *interbotix_xsturret_control* ROS package found in this layer holds the config files for every one of our X-Series turrets. These config files define the names of the joints that make up each turret as well as initial values for the motor registers. The launch file inside the package then passes the appropriate parameters to the *interbotix_xs_sdk* driver node depending on the type of turret being used.

##### Application Support Layer
The Turret module shown in this layer can be found in the *interbotix_ros_toolboxes* repository [here](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/main/interbotix_xs_toolbox). Specifically, it is located within the *interbotix_xs_modules* ROS package in a file called 'turret.py'. It essentially provides a small API to allow users to customize a sequence of pan/tilt movements - no ROS experience necessary. The *apriltag_ros* package on the other hand makes it easier to get poses from camera-detected AR tags.

##### Research Layer
All the ROS packages and Python scripts found within the [examples](examples/) directory fall in this category.

## Compatible Products
The ROS packages located here can be used with any of the Interbotix turret kits linked below. Next to each name is the name used to describe it in software (specifically for the `robot_model` argument in launch files). The software name is composed of three parts. The first two letters correspond to the model type (ex. 'wx' for 'WidowX'). The next two letters signify motor type ('xl' for XL430, and 'xm' for XM430). Finally, the last letter corresponds to the number of motors used in the tilt joint ('s' for single or 'd' for dual).
- [PhantomX XL430 Robot Turret](https://www.trossenrobotics.com/phantomx-x-series-robot-turret.aspx) (pxxls)
- [WidowX XM430 Robot Turret](https://www.trossenrobotics.com/widowx-x-series-robot-turret.aspx) (wxxms)
- [WidowX Dual XM430 Robot Turret](https://www.trossenrobotics.com/widowx-x-series-dual-servo-robot-turret.aspx) (wxxmd)
- [ViperX XM540 Robot Turret](https://www.trossenrobotics.com/viperx-x-series-robot-turret.aspx) (vxxms)
- [ViperX Dual XM540 Robot Turret](https://www.trossenrobotics.com/viperx-x-series-dual-servo-robot-turret.aspx) (vxxmd)
- [PhantomX Vision Tracking Kit](https://www.trossenrobotics.com/phantomx-x-series-robot-vision-tracking-kit.aspx) (pxxls_cam)

## Requirements
Below is a list of the hardware you will need to get started:
- Computer running Ubuntu Linux 16.04, 18.04, or 20.04 (note that virtual Linux machines have NOT been tested)
- One of the X-Series Robot Turret Kits mentioned above

## Hardware Setup
There is not much required to get the robot ready to work as most of the setup is done for you. Just make sure to do the following steps:
1. Remove the robot from its packaging and place on a sturdy tabletop surface near an electrical outlet. To prevent the robot from potentially toppling during operation, secure it to a flat surface (via clamping or using the holes on the base's perimeter). At your own risk, you could instead place a small heavy bean-bag on top of the acrylic plate by the base of the robot. Finally, make sure that there are no obstacles within the workspace of the turret.
2. Plug the 12V power cable into an outlet and insert the barrel plug into the barrel jack on the X-series power hub (located under the see-through acrylic on the base of the robot). You should briefly see the LEDs on the Dynamixel motors flash red.
3. Plug in the micro-usb cable into the U2D2 (located under the see-through acrylic on the robot's base) and your computer.

## Software Setup
To get all the code setup, refer to the computer platform types below and run the appropriate installation script. Afterwards, continue with the [Installation Checks](#installation-checks) sub-section.

###### AMD64 Architecture
If your computer uses an Intel or AMD based processor (which is the case for NUCs, most laptops and desktop computers), follow the commands below to download and run the installation script. Note that the script will also install the full desktop version of ROS (either Kinetic, Melodic, or Noetic) if it's not yet on your system. As an aside, if you've been using the old ROS packages from the [interbotix_ros_arms](https://github.com/Interbotix/interbotix_ros_arms) repository, and would like to migrate to using the ROS packages in this repository, *make sure that you do not already have a catkin workspace named 'interbotix_ws'*. Otherwise, the install script will not clone these packages. Either rename or delete the old 'interbotix_ws' workspace beforehand.

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_turrets/main/interbotix_ros_xsturrets/install/amd64/xsturret_amd64_install.sh' > xsturret_amd64_install.sh
    $ chmod +x xsturret_amd64_install.sh
    $ ./xsturret_amd64_install.sh

###### Raspberry Pi 4B (ARM64 Architecture)
If you purchased a Raspberry Pi 4B Kit with a turret from our website, there is no need to install anything as the Pi should already come preloaded with all the necessary software. If you purchased your own Raspberry Pi 4B from a third party, feel free to follow the instructions [here](https://github.com/Interbotix/interbotix_ros_toolboxes/blob/main/interbotix_rpi_toolbox/README.md) to get it properly setup before following the commands below. If you only purchased the stand-alone Raspberry Pi 4B Kit from our store (which comes pre-configured with Ubuntu and ROS), and would like to use it with an turret, then follow the commands below to download and run the installation script. Note that the script will install the full desktop version of ROS Melodic if it's not yet on your system, ask you for your desired robot model (ex. wxxmt), and prompt you about whether or not you'd like the Joystick ROS package to start at boot.

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_turrets/main/interbotix_ros_xsturrets/install/rpi4/xsturret_rpi4_install.sh' > xsturret_rpi4_install.sh
    $ chmod +x xsturret_rpi4_install.sh
    $ ./xsturret_rpi4_install.sh

If you *do* want to have the Joystick ROS package start at boot, you will first have to pair your PS4 controller with the Pi. Refer [here](https://github.com/Interbotix/interbotix_ros_toolboxes/tree/main/interbotix_rpi_toolbox#ps4-controller-setup) for details.

##### Remote Install
For some robotic projects, you may want to run your robot in a 'headless' state on some computer (like a NUC or Raspberry Pi), and monitor the robot's state (in Rviz for example) on your personal (a.k.a remote) computer over a local network. For this to work, run the installation script below on your personal Linux computer. Note that ROS and Rviz must already be installed! As an FYI, the script will prompt you to insert the hostname of the robot (NOT the remote) computer. As an example, if you wanted to monitor the state of a robot turret purchased with a Raspberry Pi 4B Kit, you would set the hostname to `pibot`. To find out the hostname of the robot computer, just open a terminal and type `hostname`

    $ sudo apt install curl
    $ curl 'https://raw.githubusercontent.com/Interbotix/interbotix_ros_turrets/main/interbotix_ros_xsturrets/install/xsturret_remote_install.sh' > xsturret_remote_install.sh
    $ chmod +x xsturret_remote_install.sh
    $ ./xsturret_remote_install.sh

Be aware that the installation script will export the ROS_MASTER_URI environment variable in your personal computer's ~/.bashrc file to `http://<hostname>.local:11311`. Make sure to comment out this line when done monitoring or your personal computer will complain about not being able to find its ROS Master.

##### Installation Checks
After running the installation script on the robot computer, verify that it was successful in finding the U2D2 by checking that the port name shows up as `ttyDXL`

    $ cd /dev
    $ ls
    $ cd

For ROS Melodic users, open the following Gazebo config file to fix an issue described [here](https://answers.gazebosim.org//question/25030/gazebo-error-restcc205-error-in-rest-request/).

    $ nano ~/.ignition/fuel/config.yaml

Now change the url inside from `https://api.ignitionfuel.org` to `https://api.ignitionrobotics.org`.

## Quickstart

1. Get familiar with the virtual robot model by launching it in Rviz and playing with the *joint_state_publisher*. Note that you must specify which turret model is being used as a command line argument. For example, the WidowX XM430 robot turret can be launched as follows:

        $ roslaunch interbotix_xsturret_descriptions xsturret_description.launch robot_model:=wxxms use_joint_pub_gui:=true

2. Get familiar with the physical robot turret (let's say... a ViperX Dual XM540!) by executing the following command in the terminal (Ctrl-C from Step 1 first):

        $ roslaunch interbotix_xsturret_control xsturret_control.launch robot_model:=vxxmd

3. By default, all the motors in the robot are torqued on so it will be very difficult to manually manipulate it. To torque off the motors, execute the command below in another terminal. Be aware though that this will cause the robot to collapse so manually hold or secure the turret before executing it.

        $ rosservice call /vxxmd/torque_enable "{cmd_type: 'group', name: 'all', enable: false}"

4. Now you should be able to freely manipulate the motors. Take note of how the Rviz model accurately mimics the real robot. To make the robot hold a certain pose, manually hold the robot in the desired pose and execute the following command:

        $ rosservice call /vxxmd/torque_enable "{cmd_type: 'group', name: 'all', enable: true}"

    You can now let go and observe how the turret stays in place.

That ends the quickstart tutorial. To get familiar with the architecture and launch file arguments, refer to the READMEs of the core packages. Start with the [interbotix_xsturret_descriptions](interbotix_xsturret_descriptions/) package, then the [interbotix_xsturret_control](interbotix_xsturret_control/) package, followed by the  [interbotix_xsturret_gazebo](interbotix_xsturret_gazebo/) package. This is the most logical approach to take to gain a better understanding of how they relate to each other. Afterwards, feel free to check out the demo projects in the [examples](examples/) directory.

## Troubleshooting
Refer to the guide [here](https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_sdk/TROUBLESHOOTING.md#troubleshooting-a-dynamixel-based-robot) to try to solve your problem. If you still need help, feel free to contact us as trsupport@trossenrobotics.com or submit an Issue. We strongly recommend the latter option though so that other people who may be facing the same difficulty can benefit. This repository is actively maintained and any open Issues will be addressed as soon as possible.

## Contributing
To contribute your own custom X-Series turret in this repo, you will need to do the following steps:
- Create a motor config file similar to the YAML files found [here](interbotix_xsturret_control/config/) (excluding the 'modes.yaml' file). To get familiar with the parameter names, checkout the [Motor Config Template](https://github.com/Interbotix/interbotix_ros_core/blob/main/interbotix_ros_xseries/interbotix_xs_sdk/config/motor_configs_template.yaml). Note that the name of this file is what defines your *robot_model* name, and should be used when naming other files like the URDF.
- Create a URDF similar in structure to the ones found [here](interbotix_xsturret_descriptions/urdf/). Don't forget to put all necessary meshes in the [meshes](interbotix_xsturret_descriptions/meshes/) directory! As an FYI, you should follow the naming convention for the links, joints, and frame poses as found in the other arm files for consistency.
- Create a set of Gazeo/ROS position controllers similar to the ones found [here](interbotix_xsturret_gazebo/config/position_controllers/).
- Make sure to follow the same naming convention, structure, and documentation procedures as found in the repo before making a PR.

## Contributors
- [Solomon Wiznitzer](https://github.com/swiz23) - **ROS Engineer**
- [Levi Todes](https://github.com/LeTo37) - **CAD Engineer**
