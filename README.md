# Racecar Competition Workspace

本工作空间是一个 ROS 小车比赛工程，核心流程分成两段：

1. 第一圈：启动车辆和传感器，运行 SLAM 建图，并可用 `explore_lite` 自动探索。
2. 第二圈：加载第一圈保存的地图，用 AMCL 定位，运行 `move_base` 导航，并可按 CSV 路点自动巡航。

主入口是 `src/racecar/launch/total.launch`。它不是直接写所有节点，而是根据参数选择 include 第一圈或第二圈的 launch 文件。

## 目录结构

```text
.
+-- README.md
+-- src
    +-- racecar
    |   +-- launch
    |   |   +-- total.launch
    |   |   +-- competition_first_lap_mapping.launch
    |   |   +-- competition_second_lap_navigation.launch
    |   |   +-- Run_car.launch
    |   |   +-- Run_gmapping.launch
    |   |   +-- move_base.launch
    |   |   +-- includes
    |   |       +-- car_tf.launch.xml
    |   |       +-- rf2o.launch.xml
    |   |       +-- amcl.launch.xml
    |   +-- map
    |   +-- param
    |   +-- src
    |   +-- save_map.sh
    +-- encoder_driver
    +-- imu_launch
    +-- lsx10 / lslidar_x10_driver
    +-- rf2o_laser_odometry
    +-- multi_goal
    +-- cam_infer
```

## 编译

在工作空间根目录执行：

```bash
catkin_make
source devel/setup.bash
```

如果是在新终端里运行 launch，必须先 source：

```bash
cd ~/racecar
source devel/setup.bash
```

## 主入口：total.launch

`total.launch` 提供比赛用统一入口：

```bash
roslaunch racecar total.launch
```

默认参数是：

```xml
first_lap=true
second_lap=false
map_file=$(find racecar)/map/mymap.yaml
use_explore=true
use_multi_goal=false
goal_csv=$(find multi_goal)/src/testceshi.csv
use_traffic_light=false
```

也就是说，直接运行 `total.launch` 默认进入第一圈建图模式。

### 第一圈：建图

```bash
roslaunch racecar total.launch first_lap:=true
```

等价于 include：

```text
racecar/launch/competition_first_lap_mapping.launch
```

启动链路：

```text
total.launch
+-- competition_first_lap_mapping.launch
    +-- Run_car.launch
    |   +-- car_tf.launch.xml
    |   +-- lslidar_x10_serial.launch
    |   +-- imu_launch/imu_spec_msg.launch
    |   +-- racecar_driver_node
    |   +-- encoder_driver/Encoder_vel.py
    |   +-- rosbridge_websocket
    +-- Run_gmapping.launch
    |   +-- rf2o.launch.xml
    |   +-- encoder_driver/launch/wheel_odom.launch
    |   +-- robot_localization/ekf_localization_node
    |   +-- gmapping/slam_gmapping
    +-- move_base.launch
    +-- explore_lite/explore_costmap.launch
    +-- racecar/car_controller_new
```

第一圈的作用：

- 启动车体底盘、雷达、IMU、编码器。
- 用 `rf2o_laser_odometry` 从 `/scan` 估计激光里程计。
- 用 `wheel_odom.launch` 和 `ekf_localization_node` 输出融合里程计 `/odometry/filtered`。
- 用 `slam_gmapping` 订阅 `/scan` 和 TF，生成 `/map`。
- 用 `move_base` 规划路径。
- 可选用 `explore_lite` 发布探索目标。
- `car_controller_new` 跟踪 `move_base` 的全局路径，最终向 `/car/cmd_vel` 发布底盘控制指令。

第一圈结束后保存地图：

```bash
bash src/racecar/save_map.sh
```

默认保存目标通常是：

```text
src/racecar/map/mymap.yaml
src/racecar/map/mymap.pgm
```

### 第二圈：定位导航

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true
```

等价于 include：

```text
racecar/launch/competition_second_lap_navigation.launch
```

启动链路：

```text
total.launch
+-- competition_second_lap_navigation.launch
    +-- Run_car.launch
    +-- map_server
    +-- rf2o.launch.xml
    +-- encoder_driver/launch/wheel_odom.launch
    +-- robot_localization/ekf_localization_node
    +-- amcl.launch.xml
    +-- move_base.launch
    +-- racecar/car_controller_new
    +-- multi_goal.launch，可选
```

第二圈的作用：

- 启动车体和所有传感器。
- 用 `map_server` 加载静态地图。
- 用 RF2O、编码器和 EKF 提供 `/odometry/filtered`。
- 用 `amcl` 在 `map` 坐标系下定位。
- 用 `move_base` 基于地图和代价地图规划路径。
- `car_controller_new` 跟踪 `/move_base/NavfnROS/plan` 并发布 `/car/cmd_vel`。
- 如果开启 `use_multi_goal`，`multi_goal` 会从 CSV 循环发布 `/move_base_simple/goal`。

指定地图：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true map_file:=$(rospack find racecar)/map/mymap.yaml
```

设置初始位姿要改 `competition_second_lap_navigation.launch` 中传给 `amcl.launch.xml` 的 `init_x`、`init_y`、`init_a` 参数，或扩展 `total.launch` 继续向下透传。

## 自动路点巡航

第二圈可以打开多目标循环：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true use_multi_goal:=true
```

默认 CSV：

```text
src/multi_goal/src/testceshi.csv
```

自定义 CSV：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true use_multi_goal:=true goal_csv:=/absolute/path/to/goals.csv
```

CSV 每行格式：

```csv
x,y,z,w
```

其中 `x,y` 是目标点位置，`z,w` 是四元数朝向的两个常用字段。`goal_loop.py` 会把这些点发布到：

```text
/move_base_simple/goal
```

## 红绿灯识别

`total.launch` 支持可选红绿灯节点：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true use_traffic_light:=true
```

这会启动：

```text
cam_infer/traffic_light.py
```

控制器 `car_controller_new` 订阅：

```text
/light
```

逻辑：

- `/light` 为 `red` 时，控制器发布停车值：`linear.x=1500`，`angular.z=90`。
- `/light` 为 `green` 或其他值时，恢复正常控制输出。

## 主要节点和话题

### 传感器和底盘

| 功能 | 节点/launch | 主要话题 |
| --- | --- | --- |
| 雷达 | `lslidar_x10_serial.launch` | `/scan` |
| IMU | `imu_launch/imu_spec_msg.launch` | 通常为 `/imu_data` |
| 编码器 | `encoder_driver/Encoder_vel.py` | `/encoder` |
| 底盘驱动 | `racecar_driver_node` | 订阅 `/car/cmd_vel` |
| WebSocket | `rosbridge_websocket` | 默认 9090 |

### 里程计和定位

| 功能 | 节点/launch | 输入 | 输出 |
| --- | --- | --- | --- |
| 激光里程计 | `rf2o_laser_odometry_node` | `/scan` | `/odom` |
| 轮速里程计 | `wheel_odom.launch` | 编码器相关数据 | odom 相关话题 |
| EKF 融合 | `ekf_localization_node` | 参数见 `ekf_params.yaml` | `/odometry/filtered` |
| SLAM | `slam_gmapping` | `/scan`、TF | `/map` |
| 定位 | `amcl` | `/scan`、`/map`、TF | `/amcl_pose`、`map->odom` TF |

注意：`rf2o.launch.xml` 中 `publish_tf=false`，所以 RF2O 不发布 `odom->base_footprint` TF，最终定位和导航主要依赖 EKF、AMCL 与静态 TF 链。

### 导航和控制

| 功能 | 节点 | 主要输入 | 主要输出 |
| --- | --- | --- | --- |
| 全局/局部规划 | `move_base` | `/move_base_simple/goal`、costmap、`/odometry/filtered` | `/move_base/NavfnROS/plan` |
| 路径跟踪 | `car_controller_new` | `/odometry/filtered`、`/move_base/NavfnROS/plan`、`/encoder`、`/light` | `/car/cmd_vel` |
| 多目标发布 | `multi_goals` | CSV、`/amcl_pose` | `/move_base_simple/goal`、`/arrfinal` |

## TF 关系

静态 TF 来自 `car_tf.launch.xml`：

```text
base_footprint -> base_link
base_link      -> laser_link
base_link      -> IMU_link
```

导航期主要 TF 链应满足：

```text
map -> odom -> base_footprint -> base_link -> laser_link
```

检查 TF：

```bash
rosrun tf view_frames
rosrun tf tf_echo map base_footprint
rosrun tf tf_echo odom base_footprint
```

## car_controller_new 控制逻辑

`car_controller_new` 是最终把规划路径变成底盘指令的节点。

订阅：

```text
/odometry/filtered
/move_base/NavfnROS/plan
/move_base_simple/goal
/encoder
/arrfinal
/line_wight
/light
```

发布：

```text
/car/cmd_vel
/car_path
```

核心逻辑：

1. 从 `/move_base/NavfnROS/plan` 获取路径点。
2. 把路径点从 `map` 坐标转换到 `odom` 坐标。
3. 在车体前方选一个距离超过 `Lfw` 的前视点。
4. 用纯追踪几何关系计算转角。
5. 用 `baseSpeed`、`vp_min`、`vp_max_base` 限制速度输出。
6. 如果收到红灯，则覆盖输出为停车值。

控制参数在第一圈和第二圈不同：

| 场景 | `Vcmd` | `baseSpeed` | `vp_min` | `vp_max_base` |
| --- | --- | --- | --- | --- |
| 第一圈 | `0.2` | `20` | `20` | `20` |
| 第二圈 | `1.0` | `150` | `380` | `300` |

代码里速度输出会加上 1500 附近的舵机/电调中值，所以实际 `/car/cmd_vel.linear.x` 常见范围是以 1500 为停车中心的 PWM 风格数值。

## 常用运行命令

第一圈建图：

```bash
roslaunch racecar total.launch first_lap:=true
```

第一圈不自动探索：

```bash
roslaunch racecar total.launch first_lap:=true use_explore:=false
```

保存地图：

```bash
bash src/racecar/save_map.sh
```

第二圈导航：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true
```

第二圈自动路点：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true use_multi_goal:=true
```

第二圈加红绿灯：

```bash
roslaunch racecar total.launch first_lap:=false second_lap:=true use_traffic_light:=true
```

手动发目标点：

```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped ...
```

查看关键话题：

```bash
rostopic list
rostopic echo /scan
rostopic echo /odometry/filtered
rostopic echo /move_base/NavfnROS/plan
rostopic echo /car/cmd_vel
```

查看节点：

```bash
rosnode list
rosnode info /move_base
rosnode info /car_controller
```

## 调试检查顺序

如果车不动，按这个顺序查：

1. `/scan` 是否有数据。
2. `/encoder` 是否有数据。
3. `/odometry/filtered` 是否持续更新。
4. `map -> odom -> base_footprint` TF 是否连通。
5. `/move_base_simple/goal` 是否有目标点。
6. `/move_base/NavfnROS/plan` 是否生成路径。
7. `/car/cmd_vel` 是否有控制输出。
8. `racecar_driver` 是否收到 `/car/cmd_vel`。
9. 如果开了红绿灯，检查 `/light` 是否一直是 `red`。

如果 `move_base` 没路径：

```bash
rostopic echo /move_base/global_costmap/costmap
rostopic echo /move_base/local_costmap/costmap
rosrun tf tf_echo map base_footprint
```

如果 AMCL 漂移：

- 确认地图和现场一致。
- 确认 `/scan` 的 `frame_id` 是 `laser_link`。
- 确认 `base_link -> laser_link` 静态 TF 与实际安装方向一致。
- 在 RViz 里重新给 `2D Pose Estimate`。

## 旧 launch 文件说明

以下文件仍然保留，部分被新比赛入口复用：

| 文件 | 说明 |
| --- | --- |
| `Run_car.launch` | 启动车体、雷达、IMU、编码器、rosbridge |
| `Run_gmapping.launch` | 启动 RF2O、轮速里程计、EKF、gmapping |
| `move_base.launch` | 启动 `move_base` 并加载 costmap/DWA/Navfn 参数 |
| `amcl_nav.launch` | 老版 AMCL 导航入口 |
| `competition_first_lap_mapping.launch` | 第一圈比赛入口 |
| `competition_second_lap_navigation.launch` | 第二圈比赛入口 |
| `total.launch` | 推荐统一入口 |

历史测试 launch 还在仓库里，但比赛运行优先使用 `total.launch`。

## 注意事项

- `total.launch` 默认只跑第一圈。如果要跑第二圈，必须显式传 `first_lap:=false second_lap:=true`。
- 不建议同时让 `first_lap` 和 `second_lap` 都为 `true`，否则会重复启动车体、雷达、EKF、`move_base` 等节点。
- `src/racecar/README.md` 里有旧说明，但当前文件编码显示有乱码；以根目录 README 为准。
- 运行前确认串口别名存在，例如 `/dev/encoder`、雷达串口、IMU 串口。
- 如果换雷达型号，需要同步修改 `Run_car.launch` 里的雷达 include 和相关 TF。
