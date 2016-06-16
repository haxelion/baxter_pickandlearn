Introduction
============

Baxter Pick and Learn is a proof of concept for an application where an operator teach a sorting task to Baxter. It uses a shape recognition algorithm to identify similar shapes and place them to a location previously demonstrated by the operator.
It was developped for a MA1 project at the VUB/ULB in collaboration with Sirris. As the objective of the project was not to develop a reliable implementation but research the possibilities offered by Baxter, this is only meant as a proof of concept and still needs a lot of improvements.

See its [Baxter wiki page](http://sdk.rethinkrobotics.com/wiki/Baxter_Pick_And_Learn) for more information.

Installation
============
It uses the Baxter SDK 0.7 and above, but has only been tested on version 0.7.
It might work up to version 1.1.0. **It is known not to work with the SDK 1.2**

``` bash
$ cd ~/ros_ws/src
$ git clone 'https://github.com/haxelion/baxter_pickandlearn.git'
$ cd ~/ros_ws
$ catkin_make
$ rosrun baxter_pickandlearn pickandlearn
```
