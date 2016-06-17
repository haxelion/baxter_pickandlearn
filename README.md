Introduction
============

Baxter Pick and Learn is a proof of concept for an application where an operator teach a sorting task to Baxter. It uses a shape recognition algorithm to identify similar shapes and place them to a location previously demonstrated by the operator.
It was developped for a MA1 project at the VUB/ULB in collaboration with Sirris. As the objective of the project was not to develop a reliable implementation but research the possibilities offered by Baxter, this is only meant as a proof of concept and still needs a lot of improvements.

See its [Baxter wiki page](http://sdk.rethinkrobotics.com/wiki/Baxter_Pick_And_Learn) for more information.

Installation
============

There are two versions in this repository, one which works for the Baxter SDK 0.7 to 1.1.1 tagged v1.1 and one for Baxter SDK 1.2 tagged v1.2.

Baxter SDK 0.7 - 1.1.1
----------------------

``` bash
$ cd ~/ros_ws/src
$ git clone 'https://github.com/haxelion/baxter_pickandlearn.git' --branch v1.1
$ cd ~/ros_ws
$ catkin_make
$ rosrun baxter_pickandlearn pickandlearn
```

Baxter SDK 1.2
--------------

``` bash
$ cd ~/ros_ws/src
$ git clone 'https://github.com/haxelion/baxter_pickandlearn.git' --branch v1.2
$ cd ~/ros_ws
$ catkin_make
$ rosrun baxter_pickandlearn pickandlearn
```

Bugs
====

Feel free to report them as an issue but please note that I don't have access to aBaxter robot anymore and that fixing them might be hard and require your help.
