# Simulation

Files relating to the simulation of the Husky.

Currently for tracking changes to code being written in Webots

### ROS

[Creating ROS nodes](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Creating-a-ROS-Package-and-Node.html)



### Webots

[Nodes and functions](https://cyberbotics.com/doc/reference/nodes-and-api-functions?tab-os=linux&tab-language=python)

[Programming docs](https://cyberbotics.com/doc/guide/programming)

[Source code](https://github.com/cyberbotics/webots_ros)

**Tutorials**

[Setting up ROS environment and running environment](https://cyberbotics.com/doc/guide/tutorial-8-using-ros?tab-os=linux&tab-language=python)

Steps to run:

* Build ROS

  ```bash
  catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
  ```

* Activate environment (Evan's install, using ZSH instead of Bash), run roscore (Local robot simulator)
  ```bash
  source ~/catkin_ws/devel/setup.zsh
  roscore
  ```
  
* Run the pioneer3at example -- ideally change this to husky

  ```bash
  rosrun webots_ros pioneer3at      
  ```


**Creating a new node**

The built in sample for ROS has an example with slam. I'm hoping to modify what's already there and adapting it to our world. To create a new node I edited the CMakeLists.txt in 

```bash
/home/evan/catkin_ws/src/webots_ros
```

to include

```c++
#instructions for husky node (Evan's modification - adapted from pioneer3at above)

add_executable(husky src/husky.cpp)

add_dependencies(husky webots_ros_generate_messages_cpp)

target_link_libraries(husky
        ${catkin_LIBRARIES}
)

```

This is a mock-up change from the pioneer3at which I've placed directly below. Also I copied pioneer3at.cpp and altered the naming to husky within.

It's necessary to recompile catkin from the root catkin_ws folder

```bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```







