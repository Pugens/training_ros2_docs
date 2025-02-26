[< Back to homepage](../README.md) \
[< Back to chapter 2](2%20-%20create%20package.md) 

<!-- omit from toc -->
# Robot End Effector

**Table of content**
- [Extending the existing robot description](#extending-the-existing-robot-description)
- [Wrist URDF Description](#wrist-urdf-description)
- [Declaring and calling a macro](#declaring-and-calling-a-macro)
- [Use a macro declared in another xacro file](#use-a-macro-declared-in-another-xacro-file)
- [Checking the resulting robot description](#checking-the-resulting-robot-description)
- [Add launch file, build and run](#add-launch-file-build-and-run)
- [Add more extensibility to the xacro file](#add-more-extensibility-to-the-xacro-file)
- [References](#references)


## Extending the existing robot description
It is possible to extend your existing URDF description by creating a Xacro file that includes and builds upon your original URDF. 

**Xacro (XML Macros)** is a ROS 2 tool that allows you to create more modular and reusable URDF files by using macros, variables, and includes. We will use this to add a wrist at the end of our 3 axis gantry setup, and then also a swappable end-effector, changing according to a given argument when launching the robot description.

- **Create a new cmake package with dependencies to the** `hello_robot` **package:**
```sh
ros2 pkg create --destination-directory 3_robot_end_effector/src --build-type ament_cmake --dependencies hello_robot --node-name my_eef my_eef
```

- **Create new folders for the URDF file, the Xacro macros to be included, the launch file, and the visual meshes for the robot description:**
```sh
mkdir -p 3_robot_end_effector/src/my_eef/urdf
mkdir -p 3_robot_end_effector/src/my_eef/urdf/inc
mkdir -p 3_robot_end_effector/src/my_eef/launch
mkdir -p 3_robot_end_effector/src/my_eef/meshes/visual
```

## Wrist URDF Description
The wrist of the gantry robot consists of 3 extra rotary motors and links attached at the end of the gantry vertical axis. They will be added to the robot description by including the existing URDF file from the `hello_robot` package.
The wrist functions like a spherical joint that can allow rotation of the end effector in all directions.

- **Create a new file for the wrist description, but in the** `xacro` **format instead of** `urdf`:
```sh
touch 3_robot_end_effector/src/my_eef/urdf/my_robot_with_wrist.urdf.xacro
```

- **Open the new file and add the following code:**

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="asd">
   <!-- include macro from other file -->
   <xacro:include filename="$(find my_eef)/urdf/inc/wrist_macro.xacro"/>

   <!-- declare local macro within this file -->
   <xacro:macro name="my_original_robot">
      <!-- add original urdf file for my_robot-->
      <xacro:include filename="$(find hello_robot)/urdf/my_robot.urdf" />
   </xacro:macro>

   <!-- call the local macro -->
   <xacro:my_original_robot />

   <!-- instantiate wrist macro from other included file with declared parameters -->
   <xacro:add_wrist  parent_link="link_3"/>
</robot>
```

This file does not contain any actual info about the robot wrist kinematic chain, nor the location of meshes. Instead, the extended xml file imports additional files needed to procedurally build a new kinematic chain.

When importing a urdf file directly, all the content is copied over without much control. But it this case, this will allow us to not repeat ourself and reuse the same description of the gantry defined in the `hello_robot` package.

## Declaring and calling a macro

- first a macro declaration must be made, giving it a name:
```xml
  ...
  <xacro:macro name="my_original_robot"> 
  ...
```
- then, the macro must be instanced by calling it by its name. Here is where its effects actually take place, by importing the original `urdf` file of the gantry robot prismatic chain:
```xml
...
<xacro:my_original_robot />
...
```

As the original file will be here processed, following any macro command, parameter of conditional function declared. In this case we declared the macro in the same file, but it usual procedure to do this in a separate file.

## Use a macro declared in another xacro file
The actual values defining the visual appearance of our attached kinematic chain are also defined in an external xacro file:

- **Create a new Xacro file in the** `/inc` **folder:**
```sh
touch 3_robot_end_effector/src/my_eef/urdf/inc/wrist_macro.xacro
```

- **Open the wrist_macro.xacro file and add the following code:**
```xml
<?xml version="1.0"?>
<robot name="wrist" xmlns:xacro="https://wiki.ros.org/xacro">
  <!-- Include the existing URDF file -->
  <xacro:macro name="add_wrist" params="parent_link">
    <link name="wrist_link_1">
      <visual>
        <geometry>
          <mesh filename="package://my_eef/meshes/visual/link_4.stl" />
        </geometry>
        <material name="magenta" />
      </visual>
    </link>
    <link name="wrist_link_2">
      <visual>
        <geometry>
          <mesh filename="package://my_eef/meshes/visual/link_5.stl" />
        </geometry>
        <material name="yellow" />
      </visual>
    </link>
    <link name="wrist_link_3">
      <visual>
        <geometry>
          <mesh filename="package://my_eef/meshes/visual/link_6.stl" />
        </geometry>
        <material name="cyan" />
      </visual>
    </link>

    <joint name="wrist_joint_1" type="revolute">
      <parent link="${parent_link}" />
      <child link="wrist_link_1" />
      <origin xyz="0 0 -0.9" rpy="0 0 0" />
      <axis xyz="0 0 -1" />
      <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1" />
    </joint>

    <joint name="wrist_joint_2" type="revolute">
      <parent link="wrist_link_1" />
      <child link="wrist_link_2" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <axis xyz="0 1 0" />
      <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1" />
    </joint>

    <joint name="wrist_joint_3" type="revolute">
      <parent link="wrist_link_2" />
      <child link="wrist_link_3" />
      <origin xyz="0 0 0" rpy="0 0 0" />
      <axis xyz="0 0 1" />
      <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1" />
    </joint>

    <!-- magenta, yellow and cyan colours -->
    <material name="magenta">
      <color rgba="1 0 1 1" />
    </material>
    <material name="yellow">
      <color rgba="1 1 0 1" />
    </material>
    <material name="cyan">
      <color rgba="0 1 1 1" />
    </material>
  </xacro:macro>
</robot>
```

This should look more familiar, apart from the line of code that must be added as a `<robot>` tag for the xml file to be condisidered as a macro by ROS:
```xml
<robot name="wrist" xmlns:xacro="https://wiki.ros.org/xacro">
...
```

Also, the macro declaration presents the most simple possible variable:
```xml
  ...
  <xacro:macro name="add_wrist" params="parent_link">
  ...
```

The first wrist end effector does not know the name of the parent link, which could be different from robot to robot. This kind of flexibility allows us to reuse this description were we required to attach the link to another kinematic chain.

The place where this parameter is used within the macro is of course at the joint declaration here:
```xml
...
<joint name="wrist_joint_1" type="revolute">
      <parent link="${parent_link}" />
      ...
```

Whereas the place where it is declared is instead at the moment of instancing of the macro, in the master description file `my_robot_with_wrist.urdf.xacro`:

```xml
...
<!-- instantiate wrist macro from other included file with declared parameters -->
<xacro:add_wrist  parent_link="link_3"/>
...
```

## Checking the resulting robot description
In case of issues with the macros results, it is possible to check the result of the xml extension by using the `xacro` command in the terminal. Simply navigate to the folder containing the file and execute the `xacro input.xacro > output.xacro` comand line: 

```sh
cd 3_robot_end_effector/src/my_eef/urdf/
xacro my_robot_with_wrist.urdf.xacro > tmp.urdf && check_urdf tmp.urdf
```

If everything works as it should, you should get a success message in the terminal with the transforms tree on the resulting urdf file:

```sh
robot name is: my_robot_with_wrist
---------- Successfully Parsed XML ---------------
root Link: base_link has 1 child(ren)
    child(1):  link_1
        child(1):  link_2
            child(1):  link_3
                child(1):  wrist_link_1
                    child(1):  wrist_link_2
                        child(1):  wrist_link_3
```

## Add launch file, build and run

Finally, we are going to add a python launch file.

- **Create a file called** `view_hello_robot_with_wrist.launch.py` **inside a** `launch` **folder.**

- **Paste the following code inside it:**

```python
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_package = FindPackageShare("my_eef")
    rvizconfig_package = FindPackageShare("hello_robot")

    description_file = PathJoinSubstitution(
        [description_package, "urdf", "my_robot_with_wrist.urdf.xacro"]
    )
    rvizconfig_file = PathJoinSubstitution(
        [rvizconfig_package, "config", "hello_robot.rviz"])

    robot_description = ParameterValue(
        Command(["xacro ", description_file]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig_file],
    )

    return LaunchDescription(
        [joint_state_publisher_gui_node, robot_state_publisher_node, rviz_node]
    )

```

- Build the package, source the installed result and launch the new extended gantry robot with wrist end effector:

```sh
colcon build --packages-up-to my_eef --merge-install --symlink-install
source install/setup.bash 
ros2 launch my_eef view_hello_robot_with_wrist.launch.py 
```

## Add more extensibility to the xacro file (extra)
As an exercise, try to think of a way to extend the wrist description with a parametric tool center point (`TCP`).

This value is very important for any robot, as it defines the transform reference from the last moving link or robot flange to the end effector acting point. This point in space, moving together rigidly attached to the last joint, can be:
- a welding torch 
- the tip of a gluing gun
- the center of a suction cup

All these tools might be present at the same time on a robot end effector, and for correct path planning and trajectory generation, you may want to be able to quickly swap from one value to another for the `TCP`.

**[Go to chapter 5 >](5%20-%20moveit2%20setup.md)**

## References
- https://articulatedrobotics.xyz/tutorials/ready-for-ros/urdf/#introducing-xacro
