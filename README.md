# Goal Biased Rapidly-Exploring Random Tree
This project demonstrates the simulation of goal biased RRT. This also retrieves the shortest path from the RRT path and then smooth the path using multinomial regression based curve fitting.

## Instructions to run the project
* First, git clone the project
```git clone https://github.com/kadupitiya/rrt.git```
* Now open a terminal(Lets call terminal 1) and navigate the project.
* Next, go to the root directory(terminal 1):
 ```cd rrt```
* Then, build the project(terminal 1):
```catkin_make```
* Then, use the following command(terminal 1):
```source devel/setup.bash```
* Next, open a separate terminal and run Ros core(terminal 2) and keep it opened:
```roscore```
* Next, run the rrt project using following command(terminal 1):
```rosrun rrt rrt```
* Finally, open another terminal and run rviz to see the visualization(terminal 3):
```rosrun rviz rviz```
* Find the “Marker” and add a "Marker". Now Rviz should start to listen to the Marker messages
published from the rrt node.
* Video:
[![Watch the video](https://github.com/kadupitiya/rrt/blob/master/images/image.jpg)](https://www.youtube.com/watch?v=SZ9c_3HUVUE&t=5s)
