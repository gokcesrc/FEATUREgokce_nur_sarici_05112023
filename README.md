# feature/gokce_nur_sarici_05112023
# WHAT IS ROS?

ROS stands for "Robot Operating System." Contrary to its name, ROS isn't strictly an operating system. Instead, it's a flexible framework or middleware designed for writing software for robots. ROS provides a structured series of tools, libraries, and conventions to simplify the complex process of robot software development.

- **Hardware Abstraction**: ROS provides a standardized way to interface with different robot hardware components, allowing for easier integration and interchangeability.

- **Software Reusability**: ROS promotes the development of reusable software packages, which can be utilized across different robot projects.

- **Message-Based Communication**: ROS uses a publish-subscribe and service-client paradigm for inter-process communication, enabling different software components (nodes) to communicate and exchange data.

## Publish-Subscribe Model

- The publish-subscribe model is a messaging communication pattern used in distributed systems. In this model:
  - Publishers produce and send messages without specifying which subscribers, if any, will receive them.
  - Subscribers express interest in one or more topics and only receive messages that are of interest, without knowing which publishers, if any, there are.

- Key features of the publish-subscribe model:  
  - **Decoupling**: Publishers and subscribers are decoupled, meaning they don't need to know about each other's existence. This allows for flexibility in system evolution.
  - **Dynamism**: Subscribers can join or leave at any time, and publishers can come online or go offline without affecting the system significantly.
  - **Scalability**: Supports a large number of publishers and subscribers, making it scalable for broad applications.
  - **Flexibility**: Subscribers receive only the messages they're interested in, based on the topics or criteria they've subscribed to.

### Component-Based Approach

- ROS encourages breaking down robot software into modular components (known as nodes). These nodes can communicate with each other using the ROS communication infrastructure.

### Extensive Libraries and Tools:
- ROS offers libraries and tools for common tasks in robotics, such as motion planning, perception, and sensor data integration.

### Community Support:
- Supported by an active community, there's a wealth of tools, libraries, and packages developed and shared by the community for various robotic applications.

### Support for Multiple Languages:
- ROS primarily supports C++ and Python, allowing developers to write nodes in either or both of these languages.

### Simulation Capabilities:
- With tools like Gazebo, ROS can simulate a robot in a 3D environment, allowing for testing of robot software without requiring physical hardware.


### Why Linux?
- ROS works better on Linux because it was initially designed and optimized specifically for this operating system. Additionally, the open-source nature and customizability of Linux allow users and developers to easily tailor and optimize ROS to their specific needs.
0
# WORKING PRINCIPLE OF ROS

![ROS File System Level Diagram](URL-of-the-image)

- **Meta Packages:**
  - These are groupings of multiple packages. They are particularly used in larger projects or when bundling a series of packages together.
  
- **Packages:**
  - The basic unit of ROS applications. A package can contain ROS nodes, libraries, datasets, configuration files, etc. It typically represents an independent functionality.
  
- **Package Manifest:**
  - An XML file found in the root directory of every ROS package that contains meta information about the package. It defines the dependencies, version, author, license, and other details of the package.

- **Messages:**
  - Data structures used to define the information exchange between ROS nodes. By creating custom message types, you can exchange information about the robot's sensors, movements, or other data.

- **Services:**
  - Used in ROS to send request/response messages from one node to another in a synchronized manner. A service defines two types of messages, one for the request and one for the response.

- **Codes:**
  - Represent the codes contained within the packages. These can be ROS nodes, libraries, or helper utilities.

- **Misc (Miscellaneous):**
  - Represents other components that might be present in a ROS package like configuration files, parameter files, setup scripts, etc.

*This diagram illustrates how ROS is organized and how its fundamental components come together. Due to ROS's modular nature, if you wish to add or modify a certain functionality or feature, you can simply modify or add the relevant package or component to achieve that functionality.*

FOTO
- **CMakeLists.txt**:
  - Contains information on how the code should be compiled. It provides configuration and compilation directives using the CMake tool.

- **package.xml**:
  - An XML file format that contains meta information about the package. It defines the package's dependencies, author, version, and licensing details.

- **config**:
  - Contains configuration files.

- **include**:
  - The folder containing header files.

- **scripts**:
  - Contains various scripts, usually where Python codes are placed.
    
- **src**:
  - Contains the source codes, where C++ codes are typically placed.

- **launch**:
  - Contains launch files used to start ROS nodes.
    
- **msg**:
  - Contains custom message definitions.

- **srv**:
  - Contains service definitions.

- **action**:
  - Contains definitions for actions.
 
FOTO

- **Nodes**:
  - These are the individual processes that perform computations in ROS.
  - Each node is designed to perform a specific task, such as controlling a motor, reading from a sensor, or processing data.
  - Nodes communicate with each other through topics, services, and sometimes through the Parameter Server.

- **Master**:
  - The ROS Master provides name registration and lookup services to the rest of the nodes in the system.
  - It's crucial for enabling individual nodes to locate one another.
  - Essentially, it's the central hub of the ROS communication system.

- **Parameter Server**:
  - It's a shared, multi-variate dictionary that nodes can use to store and retrieve parameters at runtime.
  - This is useful for globally storing values like configuration parameters that multiple nodes might need to access.

- **Messages**:
  - These are the data structures that nodes use to communicate with each other.
  - They can be simple (e.g., an integer or a float) or complex (e.g., arrays, fixed-size arrays, constants)
 
- **Topics**:
  - Channels that nodes use to send and receive messages.
  - Nodes "publish" to a topic or "subscribe" to a topic to receive information.
  - For example, a camera node might publish images to a "camera/image" topic, and any node that wants to access these images would subscribe to that topic.

- **Services**:
  - A more synchronous way for nodes to communicate, compared to topics.
  - One node offers a service, and another node calls it, typically expecting a reply.
  - This is similar to the client-server model; the node providing the service is the server, and the node calling the service is the client.

- **Bags**:
  - ROS's way of storing and playing back ROS message data.
  - Useful for saving data like sensor readings and then replaying it for testing and analysis.
  - The arrows in the diagram indicate the flow of communication and dependencies between these components. The overall architecture is designed to be flexible and distributed, allowing for modular robotics development and system design.
 
    FOTO

- **ROSMASTER**:
  - Roscore is a central component of ROS (Robot Operating System).
  - Roscore needs to be running for ROS to start.
  - roscore launches the ROS master, which is the communication hub of ROS, as well as a parameter server and a logging component called rosout.
  - The ROS master enables ROS nodes to communicate with each other.
  - The parameter server stores parameters that are shared between nodes.
  - The rosout logging component collects log messages from nodes and prints them to the console.
  - When roscore is started, ROS nodes can communicate with each other and use parameters stored in the parameter server.

- **Publisher-Subscriber Model**
  FOTO
- **Server-Client Model**
- foto

  ## CHECKPOINT II

- **Which Distribution Should I Use?**
  - ROS Melodic Morenia (LTS) supported until May 2023, recommended for Ubuntu 18.04
  - ROS Noetic Ninjemys (latest LTS) supported until May 2025, recommended for Ubuntu 20.04

## CHECKPOINT III

- **Creating Workspace**
  - **What is Catkin?**
    - Catkin is the most widely used build system within ROS, utilized for building ROS packages and software.
    - The term `catkin_ws` typically refers to the name of a Catkin workspace, which is the directory where ROS packages are developed, built, and maintained together.
    - A workspace contains folders that store source code, compile processes, and the executable files that are created. `catkin_ws` commonly has the following structure:
      - `src/`: This is where your source files (for example, C++ or Python source code) are located. Your ROS packages would be here.
      - `build/`: When you run `catkin_make` or `catkin build` commands, Catkin creates intermediate files during the build process in this directory.
      - `devel/`: After the build process, the executables and other products of the project are placed in this directory. The `devel/` directory also contains the environment settings required for runtime.

```
$ mkdir -p catkin_ws/src
$ cd catkin_ws
$ ls
build  devel  src
$ catkin_make
Base path: /home/ubuntu/catkin_ws
Source space: /home/ubuntu/catkin_ws/src
Build space: /home/ubuntu/catkin_ws/build
Devel space: /home/ubuntu/catkin_ws/devel
Install space: /home/ubuntu/catkin_ws/install
####
Running command: "make cmake_check_build_system" in "/home/ubuntu/catkin_ws/build"
####
####
Running command: "make -j3 -l3" in "/home/ubuntu/catkin_ws/build"
```

The `catkin_ws` is typically created and used with the following commands:
- `mkdir -p ~/catkin_ws/src`: To create a Catkin workspace and a `src` directory within it.
- `cd ~/catkin_ws/`: To switch to your created workspace.
- `catkin_make`: This command is used to initially build your workspace or to rebuild your workspace after making changes to your source code.

ROS users often work on multiple projects or robotic systems, so they may create different workspaces for different projects. This allows them to keep dependencies and configurations separate between projects.

### Source the environment:
- Before using the workspace, you need to source the setup file to set the right environment variables.
  - You'll need to do this in every new terminal you open before you can use the ROS commands that depend on your workspace.

- If you don't want to source the workspace setup file every time;
```
$ source ~/catkin_ws/devel/setup.bash
$ gedit ~/.bashrc
```
- Afterward, you can add the path highlighted in yellow to your .bashrc file.
  FOTO

# Navigating the Filesystem in ROS

## How to Find a Package in ROS?

- To easily navigate to the desired directory of a ROS package, use the `roscd` command:

    ```
    $ rospack find rospy
    /opt/ros/melodic/share/rospy
    $ roscd rospy
    ```

- To navigate to the subdirectories of that file, you can use the following command;
    ```
    $ ls
    cmake package.xml rosbuild
    $ roscd rospy/cmake
    ```

- If you want to list the files inside the directory without actually navigating to it, we can use the following command;

    ```
    $ cd ~
    $rosls rospy
    cmake package.xml rosbuild
    ```


## CHECKPOINT IV

## CHECKPOINT V

### Creating a ROS Package

To create a ROS package, follow these steps:

```
$ cd catkin_ws/src
$catkin_create_pkg composiv_tryouts
```

FOTO1
FOTO2
FOTO3

### After creating the package, don't forget to configure it with `catkin_make`.

```
$ rosls composiv_tryouts
CMakeLists.txt include launch msg package.xml scripts src srv
cd ~
$ cd catkin_ws
$ catkin_make
```

## CHECKPOINT VI

- Implementing a publisher and subscriber
- Firstly let's create the script file.

```bash
$ cd ~
$ roscd composiv_tryuts
$ mkdir scripts
```

-  Use the gedit command to write our talker code.
```
$ cd scripts
$ gedit composyt_talker.py
```

foto1
```
$ gedit composiv_listener.py
```
foto2

- To make the files we created executable, we need to change their permissions with chmod.

```
$ chmod +x *
$ rosrun composiv_tryouts composiv_talker.py
```
foto

```
$ rosrun composiv_tryouts composiv_listener.py
```
foto

## CHECKPOINT VII

- **Launch Folder**
  - Roslaunch is a launch file that allows us to run multiple packages.

```
$ roscd composiv_tryouts/launch
$ ls
composiv_tryout.launch
$ gedit composiv_tryout.launch
```
foto

```
$ roscd composiv_tryouts/launch
$ roslaunch composiv_tryouts composiv_tryout.launch
```
FOTOSUNU KOY CODE CIKTISININ

## REFERENCES

- **Virtual Machine and Ubuntu Installation**
  - Tutorial: [https://www.youtube.com/watch?v=BkXit-KHV5E](https://www.youtube.com/watch?v=BkXit-KHV5E)
  - VirtualBox: [https://www.virtualbox.org/wiki/Downloads](https://www.virtualbox.org/wiki/Downloads)
  - Ubuntu 18.04: [https://releases.ubuntu.com/18.04/](https://releases.ubuntu.com/18.04/)
    - **ROS Melodic Morenia:** [https://wiki.ros.org/melodic](https://wiki.ros.org/melodic)
  - Ubuntu 20.04: [https://releases.ubuntu.com/20.04/](https://releases.ubuntu.com/20.04/)
    - [https://wiki.ros.org/noetic/Installation](https://wiki.ros.org/noetic/Installation)

- **Ros Installation and Distribution**
  - **ROS Distributions:** [http://wiki.ros.org/Distributions](http://wiki.ros.org/Distributions)
  - **ROS Installation:** [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation)

- **Wiki ROS Tutorials:**
  - **Wiki ROS:** [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)

- **Websites where you can ask questions related to ROS:**
  - **ROS Questions:** [https://answers.ros.org/questions/](https://answers.ros.org/questions/)
  - **Gazebo Questions:** [http://answers.gazebosim.org/questions/](http://answers.gazebosim.org/questions/)

















