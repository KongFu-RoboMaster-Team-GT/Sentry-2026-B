# Livox mid 360 ladar devops log

#### 1. 目标（Goal Setting）：我们现在要干什么？


1. when boostraping gazebo, the robot model is missing. this is a path issue.
  solution: (need verify) the example [urdf tutorial](https://github.com/ros/urdf_tutorial?tab=readme-ov-file) works normally, but it lies on ros system dir, not user dir.
we can refer above example and try to migrate them to user dir.


2. gazebo sim ros controller incomplete. I add gazebo plugin but not add ros2_control plugin yet.

3. when using `launchemptyworld.py`, modified the `/models/arena3v3.world` file.
search for this line: `<uri>/home/phage/Downloads/RMUL2026.stl</uri>`
replace it with `/home/your usr name!!!!/path/to/rm_sim_bringup/models/RMUL2026.stl`

Need to figure how to solve this stupid path problem, since target 1 sucks because same path problem.....

##### 2. 核心决定（Decision Log）：我们决定了什么？

for target 1, I've try modified `package.xml`. this could be the problem
we should check how does the xacro indexing the stl's. this coule also be the path problem

current launch script status:
`/launch/eval_robot_only.py` bootstrap the world with a robot, but it is broken.
you probably see gazebo stuck at loadup, consider use headless to improve this.
(because parts missing happens even when I run official examples, strange)

`launchemptyworld.py` relys on putting

3. target 3 I already put stl under rm_sim_bringup

##### 3. 研发过程（Development Log）：我是怎么做到的？

an ros2 module install process (use livox mid360 as example)
go to :(https://github.com/Livox-SDK/livox_ros_driver2)
official command is `git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2`
but don't follow it. the above command copy copy the repo to a wrong directory. you need to copy the repo to `ros2_user_space/src/` 

then follow the repo instruction to install it. remember using ros2 humble instructions!
for the ip addr, config it. for host machine change your eth card, for ladar, the ip addr should be 192.168.1.1xx, xx is last two digit of broadcast code



---

##### 4. 理论知识（Knowledge Base）：我学会了什么？


ros misc:
when you not bootstrap any node, there are two default topics: 
`/rosout` it is ros2 system logging
`/parameter_events` parameters notification center. 


Gazebo:
urdf: used by ROS2 to descript movement of robot, part of robot OS
urdf just a resources map, it descript the location of the parts `stl` file



---

##### 5. 链接（References）：资源都在哪里？


ros2 simple intro:
https://docs.ultralytics.com/zh/guides/ros-quickstart/

ros2 structure explain:
https://zhuanlan.zhihu.com/p/670462456



ros2 complete introduction (third party)
https://zhuanlan.zhihu.com/p/639234090
这教程卖课味冲我脸上了都, 但是写的还不错其实
ros2 node explain:
https://zhuanlan.zhihu.com/p/639240328

ros2 official cn docs
http://fishros.org/doc/ros2/humble/Tutorials/Beginner-CLI-Tools/Launching-Multiple-Nodes/Launching-Multiple-Nodes.html


gazebo:

short intro to gazebo: https://bbs.huaweicloud.com/blogs/284569

gazebo models:
https://app.gazebosim.org/dashboard

misc:
https://zhuanlan.zhihu.com/p/367660310


possible solutions:
https://github.com/ros/urdf_tutorial?tab=readme-ov-file





---

##### 6. 反思（Reflection/Inspiration）：意外的收获和想法

**模板包含：一个灵活的思考空间**

这个区域是留给那些“**灵光一闪**”的想法。

*   你可能在调试 A 功能时，突然想到 B 功能的一种全新实现方式，但现在还不是实现它的时机。
*   你可能对项目的整体架构产生了新的质疑或更宏大的规划。
*   你甚至可以写下对整个开发过程的感受：哪个环节做得不好，下次要如何改进。

“反思”鼓励你跳出当前任务的局限，进行批判性思考或创新思维捕获。这些想法往往是未来项目或个人成长的起点。

