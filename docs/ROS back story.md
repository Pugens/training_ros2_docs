[<= Back to homepage](../README.md) 

# What is ROS?
ROS is a free and open-source software (**FOSS**), often referred to as a robotic middleware suite. ROS is not an operating system (**OS**) per se, but a set of software libraries and frameworks for robot software development. It provides components designed for a heterogeneous set of uses, such as hardware abstraction, low-level device control, machine vision and common robotic programming functionality, message queueing between processes, package management and compiling tools [^wikipedia_ros].

## Brief history
ROS initially started as an academic project in 2007 at the Stanford Artificial Intelligence Laboratory [^wizards-of-ros-willow]. It aimed to accelerate research and development by providing a common platform for sharing code and algorithms among roboticists and researchers. Over time, ROS gained popularity due to its flexibility, modular architecture, and extensive libraries, mainly thanks to the community enthusiastic support, the featuring of cutting-edge research from academia, and the extension to further and further platforms and sensors supported by the single users.

Main robotics applications in industrial automation involved just industrial manipulators (robotic arms) and other hard automation (CNCs, conveyors, machinery) for the longest time and still today. While ROS was not originally intended for industrial use, the industrial automation sector soon recognized its value, with projects like [ROS Industrial](https://rosindustrial.org) emerging to support further development of its robustness and to extend its advanced capabilities to relevant industrial hardware and applications.

## Not just industrial robots
With the help of ROS navigation stack (https://www.opennav.org), ROS facilitates control, mapping and coordination of Automated Guided Vehicles (AGVs) and mobile robots (MR), in any sort of factory environments. Logistics and warehouses is the sector where this made the greatest impact. The same goes for search&rescue, cleaning industry, agriculture and transportation.


## The hardware abstraction
ROS allows easy integration of various sensors, such as cameras, LiDAR, and force/torque sensors, into robotic systems. This is thanks to the Data Distribution Service (DDS), a connectivity framework middleware standard that creates reliable and performant, real-time communications queues between publishers and subscribers[^wikipedia_dds] [^ros_on_DDS]. But the main reason the integration is so easy and smooth within ROS ecosystem is thanks to the extensive collection of independently developed drivers by the great community using and backing the project.
Many of such libraries and extensions enable communication support with most common industrial devices and networks, like OPC-UA, MqTT, ZeroMQ, Arduinos, etc. etc.

## The community
In summary, ROS transitioned from academia to industry, and its adoption has been instrumental in advancing robotics capabilities across various technical domains and industries. It empowers developers to transition research innovations from the lab to real-world applications on the factory floor. An for theses reasons, it is supported by large communities online, all contributing to the same effort of making robotics open to everyone.

For more support, please visit:
- https://docs.ros.org/en/rolling/Contact.html
- https://discourse.ros.org
- https://robotics.stackexchange.com
  

[Go to installation instruction ->](Installation.md) 

# References
[^wikipedia_ros]: [Source: Wikipedia at *https://en.wikipedia.org/wiki/Robot_Operating_System*](https://en.wikipedia.org/wiki/Robot_Operating_System)
[^wizards-of-ros-willow]: [*"Wizards of ROS: Willow Garage"* (IEEE Spectrum article)](https://spectrum.ieee.org/wizards-of-ros-willow-garage-and-the-making-of-the-robot-operating-system)
[^wikipedia_dds]: [Source: Wikipedia at *https://en.wikipedia.org/wiki/Data_Distribution_Service*](https://en.wikipedia.org/wiki/Data_Distribution_Service)
[^ros_on_DDS]: [ROS on DDS at *https://design.ros2.org/articles/ros_on_dds.html*](https://design.ros2.org/articles/ros_on_dds.html)