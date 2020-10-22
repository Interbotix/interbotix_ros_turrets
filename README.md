![turret_banner](images/turret_banner.png)

## Overview
![turret_repo_structure](images/turret_repo_structure.png)
Welcome to the *interbotix_ros_turrets* repository! This repo contains custom ROS packages to control the various types of turrets (a.k.a pan/tilt mechanisms) sold at [Interbotix](https://www.trossenrobotics.com/). These ROS packages build upon the ROS driver nodes found in the [interbotix_ros_core](https://github.com/Interbotix/interbotix_ros_core) repository. Support-level software can be found in the [interbotix_ros_toolboxes](https://github.com/Interbotix/interbotix_ros_toolboxes) repository.

## Repo Structure
```
GitHub Landing Page: Explains repository structure and contains a single directory for each type of turret.
├── Turret Type X Landing Page: Contains 'core' turret ROS packages.
│   ├── Core Turret ROS Package 1
│   ├── Core Turret ROS Package 2
│   ├── Core Turret ROS Package X
│   └── Examples: contains 'demo' turret ROS packages that build upon some of the 'core' turret ROS packages
│       ├── Demo Turret ROS Package 1
│       ├── Demo Turret ROS Package 2
│       ├── Demo Turret ROS Package X
│       └── Python Scripts: contains 'demo' Python scripts that build upon modules in the interbotix_ros_toolboxes repository
│           ├── Demo Python Script 1
│           ├── Demo Python Script 2
|           └── Demo Python Script X
├── LICENSE
└── README.md
```
As shown above, there are five main levels to this repository. To clarify some of the terms above, refer to the descriptions below.

- **Turret Type** - Any robotic turret that can use the same *interbotix_XXturret_control* package is considered to be of the same type. For the most part, this division lies on the type of actuator that makes up the robot. As an example, all the X-Series turrets are considered the same type since they all use various Dynamixel X-Series servos (despite the fact that they come in different sizes and motor versions). However, a turret made up of some other manufacturer's servos, or even half made up of Dynamixel servos and half made up of some other manufacturer's servos would be considered a different type.

- **Core Turret ROS Package** - This refers to 'High Profile' ROS packages that are essential to make a given turret work. Examples of 'High Profile' ROS packages include:
    - *interbotix_XXturret_control* - sets up the proper configurations and makes it possible to control the physical turret
    - *interbotix_XXturret_gazebo* - sets up the proper configurations and makes it possible to control a Gazebo simulated turret
    - *interbotix_XXturret_descriptions* - contains URDFs and meshes of the turrets, making it possible to visualize them in Rviz

- **Demo Turret ROS Package** - This refers to demo ROS packages that build upon the **Core Turret ROS Packages**. ROS researchers could use these packages as references to learn how to develop their own ROS packages and to get a feel for how the robot works. Typical demos for a given turret type include:
    - *interbotix_XXturret_simple_interface* - use a GUI or joystick controller to manipulate a turret
    - *interbotix_XXturret_object_tracker* - perform object tracking using a USB camera

- **Demo Python Script** - This refers to demo Python scripts that build upon modules in the *interbotix_ros_toolboxes* repository. These modules essentially abstract away all ROS code, making it easy for a researcher with no ROS experience to interface with a turret as if it was just another Python object. It also makes sequencing robot motion a piece of cake.

Over time, the repo will grow to include more types of turrets.

## Contributing
Feel free to send PRs to add features to currently existing Turret ROS packages or to include new ones. Note that all PRs should follow the structure and naming conventions outlined in the repo including documentation.

## Contributors
- [Solomon Wiznitzer](https://github.com/swiz23) - **ROS Engineer**
- [Levi Todes](https://github.com/LeTo37) - **CAD Engineer**
