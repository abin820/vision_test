# 27算法组导航方向招新大作业

## 0\. 环境配置

```Plain Text
// 已装 Ubuntu 22.04 和 ROS2 Humble
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-pluginlib \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-std-srvs \
  ros-humble-rviz2 \
  ros-humble-behaviortree-cpp-v3 \
  qtbase5-dev \
  libopencv-dev \
  libyaml-cpp-dev
  
pip3 install --user pygame numpy Pillow
```

## 1\. 作业目标

在给定的迷宫地图上，完成 **一次点击导航** 全流程：

1. 机器人从 **固定起点** 出发；

2. 选手在 RViz 中对 **指定终点** 发布一次目标；

3. 上层行为树 `click_nav.xml` 触发导航；

4. 下层行为树 `default_nav_with_fallback.xml` 调用 A\* 规划全局路径；

5. **你实现的控制器插件** 跟踪规划轨迹，使机器人准确、平滑地到达终点。

## 2\. 任务场景

### 2\.1 地图

|项目|数值|
|---|---|
|文件|`maze_map.pgm` / `maze_map.yaml`|
|分辨率|`0.05 m/pixel`|
|尺寸|`15.00 m × 15.00 m`（300×300）|
|走廊宽度|约 `1.50 m`|
|墙厚|约 `0.15 m`<br>|

<p align="center">
  <img src="images/image_1.png" alt="仿真 GUI" width="420" />
  <img src="images/image_2.png" alt="代价地图与全局路径" width="420" />
</p>

### 2\.2 固定起终点

||**map 系坐标 \(x, y\)**|**说明**|
|---|---|---|
|**起点**|`(0.900, 0.900)`|迷宫左下角通路中心；仿真初始位置已设为此处|
|**终点**|`(14.100, 14.100)`|迷宫右上角通路中心；评测时只允许对该点 click 一次|

- **不允许拖拽改起点、不允许中途二次改目标；**

- **不允许手动发布 ****`cmd_vel`**** 绕过控制器。**

### 2\.3 仿真器

- **不允许修改仿真器内部代码及参数；**

- **仿真里机器人撞墙便会卡住。**

## 3\. 发放内容与你需要完成的部分

### 3\.1 发放给你的代码

|包|内容|
|---|---|
|`robot_msg`|消息 / Action|
|`sp_map_server`|ESDF / 全局代价地图|
|`sp_global_planner`|A\* 全局规划|
|`sp_nav_bt`|下层 BT：`default_nav_with_fallback.xml`|
|`sp_decision`|上层 BT：`click_nav.xml`|
|`sp_nav_bringup`|启动与地图（含 `maze_map`）|
|`sp_nav_sim`|拖拽仿真（里程计 \+ TF）|
|`sp_controller_server`|控制器框架（`ControllerPlugin` 接口、`controller_node`、pluginlib 加载）|

控制器侧（名称可自定，但须 pluginlib 可加载）：

- `include/sp_controller_server/controller_plugin.hpp`（接口，只读）

- `plugins/<你的控制器>.hpp/.cpp`（由你实现）

- `plugin_description.xml`、`CMakeLists.txt`（需正确注册插件）

- `launch/controller.launch.py`

**注意：代码中已给出 PidController（\.cpp 与 \.hpp）的大体框架，可直接在此基础上修改。**

### 3\.2 你必须完成的工作（代码中已用 TODO 标明）

1. **实现路径跟踪控制器插件**

实现 `ControllerPlugin` 三个接口：

- `configure(...)`：读参数、初始化

- `setPlan(path)`：接收 `/global_path`（经 controller server 转发）

- `computeVelocityCommands(pose, velocity)`：输出 `TwistStamped`（线速度）

2. **修改参数使工程能在迷宫上跑通**

发放包中部分路径 / 话题 / 坐标系 / 为空，直接运行`sim_nav_all_start.sh`不能正确评测迷宫任务，你需要自行排查并修改。

3. **可视化与代码管理**

在 RViz 中至少清晰展示：全局代价地图、全局路径、机器人位姿、目标点等；代码要有清晰的 commit 记录，知道该忽略哪些文件。

## 4\. 评分标准（100 分）

**评测方式：固定起点 → 一次 click 到固定终点 → 记录从目标下发到判定到达的过程。**

|**分项**|**分值**|**考察内容**|
|---|---|---|
|**准确到达**|25|最终位置与终点距离|
|**时间**|25|从收到目标到到达的用时|
|**跟踪精度**|20|对采样位姿计算到全局参考路径的误差|
|**报告**|15|清晰的 README 介绍你的项目|
|**可视化与代码管理**|15|RViz 地图、路径、机器人清晰；代码管理清晰|

## 5\. 加分项（**不超过总分 100**）

- 轨迹平滑（平滑 A\* 轨迹或直接使用其他算法）；

- 自研优于 PID 的控制器（例如使用 LQR 或 MPC）。

## 6\. 提交要求

- 线上提交到自己的 git 仓库对应分支，需包含完整可编译源码；

- 如虚拟机卡顿且自己无法解决，10\.6后可线下到地下室使用小电脑调试（需提前告知群里导航方向管理员）；

- DDL：10\.25。



