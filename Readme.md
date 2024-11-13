## 功能包位置

```
/ht_MTraMap_for_planning_scout_ws
---/src
------/controllors_pkg
```



## 编译

```
catkin_make -DCATKIN_WHITELIST_PACKAGES="controllors_pkg"
```



## 启动

```
roslaunch controllors_pkg ctrl2_kinemic.launch
```



首先启动 地图的launch文件

然后启动 规划的launch文件

最后启动 控制的launch文件

规划器发布路径后控制器会直接开始工作。

目前控制器还有点问题，最后一个路径点会差一点才跟上。路径的大部分都能跟踪。

使用的文件包含：

```
LQR.hpp
scout_define.hpp
scout_ctrl_2_kinemic.cpp
```

