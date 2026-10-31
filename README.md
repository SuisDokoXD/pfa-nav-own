## 编译
[运行以下指令编译代码]

```bash
colcon build --symlink-install --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 进入工作区

编译成功后：
以下所有命令都需要cd后进入了工作区主要目录之后,执行以下命令才能生效:

```bash
source install/setup.bash
```

## 仿真命令

1.仿真世界启动：--------------------------------------------------
[首先明确，只有在仿真状态下才需要启动仿真世界，实车状态下直接启动导航即可]

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py
```

2.仿真建图：---------------------------------------------------

(1) 仿真建图直接启动的指令
[需要配合仿真世界才能使用]

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py slam:=True
```

(2) 仿真先验导航直接启动的指令
[需要配合仿真世界才能使用]
{会从 src/pb2025_sentry_nav/pb2025_nav_bringup 下读取地图和点云}

```bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py world:=rmuc_2025 slam:=False
```

(3)一键仿真建图并覆盖仿真先验导航的 `game` 地图
[仿真导航传参的地图和点云不见了使用，创建新的点云和栅格，然后自动导入]
[默认 Gazebo 起 `rmuc_2027`，建完装成 `game.*`；位置参数只决定 Gazebo 场地，安装名由 `INSTALL_NAME` 决定，给别的场地存档要显式传 `INSTALL_NAME=<场地名>`]

```bash
# 默认：Gazebo 起 rmuc_2027，结果装成 game.yaml/game.pgm/game.pcd
./tools/match_start/simulation/sim_slam_install_map.sh

# 想按场地名存档（不覆盖 game）：
INSTALL_NAME=rmuc_2027 ./tools/match_start/simulation/sim_slam_install_map.sh rmuc_2027
```

脚本运行逻辑：
先用 `rmuc_2027`（或你传的第一个位置参数）开Gazebo，再开仿真SLAM，自己点点跑完需要建图的区域后按 `Ctrl+C`关闭它，关闭后，slam会保存 `.pgm/.yaml`，point_lio会保存`.pcd`，然后统一改名并覆盖安装到以下目录（`game` 就是 `INSTALL_NAME` 的默认值）：

```text
src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/game.yaml
src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/game.pgm
src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/scans.pcd
src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/simulation/game.pcd
```
(4)一键启动仿真全套指令
Gazebo + 仿真先验导航模式：
[Gazebo 默认起 `rmuc_2027`，导航默认读取 `world:=game`，即刚才覆盖安装的 `game.yaml` 和 `pcd/simulation/scans.pcd`；要换导航地图用 `NAV_WORLD=<场地名>`。]

```bash
# 等价于 ./tools/match_start/simulation/sim_prior_nav_game.sh rmuc_2027
./tools/match_start/simulation/sim_prior_nav_game.sh
```

(5)在仿真下手动键盘控制车移动

```bash
ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=5.0 -p w:=0.3
```

(6)手动保存栅格地图
[保存的是yaml和pgm，不保存点云]

```bash
ros2 run nav2_map_server map_saver_cli -f <YOUR_MAP_NAME> --ros-args -r __ns:=/red_standard_robot1
```

## 实车建图
1.直接执行实车建图的指令
[在正常启动哨兵后，在哨兵小电脑运行以下指令，直接启动建图导航]

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True
```

单独运行导航工作区时，建图和先验导航都要传 `use_robot_state_pub:=True`。该参数默认是 `False`，只有完整机器人系统已经在相同命名空间发布 `base_footprint`、`front_mid360`、`gimbal_yaw` 的 TF 时才可以省略或设为 `False`，避免重复发布 TF。缺少这些 TF 会导致 `loam_interface` 一直重试，随后 Nav2 一直等待 `gimbal_yaw_fake -> odom`。

2.一键启动实车建图以及目标点管理工具

(1)打开实车在线建图、Nav2、RViz 和目标管理小工具：

```bash
./auto_nav/start_reality_online_slam_nav.sh
```

(2)仿真模式下通过传参使用已经保存的实车目标点自动导航
[目标文件位于 `online_slam_goals/reality/` ]

```bash
./auto_nav/start_reality_online_slam_nav.sh --goal <实车目标名称>
```

{正式比赛模式默认启用启动门控：即使地图、TF 和 Nav2 已准备完成，也必须收到 `/referee/game_status` 的 `pb_rm_interfaces/msg/GameStatus` 消息，且 `game_progress=RUNNING(4)` 后才发送目标。实际裁判话题名称不同时，使用 `--game-status-topic <话题>` 指定。}

(3)实车调试下通过传参使用已经保存的实车目标点自动导航
[实车调试时必须显式增加 `--test-mode`。小窗口会再次要求安全确认，确认后不等待裁判系统信号]

```bash
./auto_nav/start_reality_online_slam_nav.sh --test-mode --goal <实车目标名称>
```

(4)实车调试下通过传参使用已经保存的实车目标路线自动导航
[小工具支持将多个已录制单点可视化排序并保存为 `pfa_online_slam_route/v1` 顺序路线。路线文件保存在对应环境的 `routes/` 子目录]

```bash
./auto_nav/start_reality_online_slam_nav.sh --test-mode --route <实车路线名称>
```

(5)正式比赛时通过传参使用已经保存的实车目标路线自动导航
[正式比赛执行路线时去掉 `--test-mode`，整条路线会等待比赛 `RUNNING` 信号后才开]

```bash
./auto_nav/start_reality_online_slam_nav.sh --route <实车路线名称>
```

(6)查看可用目标名称

```bash
./auto_nav/start_reality_online_slam_nav.sh --list-goals
```
-------------------------------------------------------------------------------
脚本的实车默认值为空命名空间、`use_sim_time=False`，在线建图时 RViz Fixed Frame 自动使用 `odom`。如果完整机器人系统已经在发布关节 TF，请增加 `--no-robot-state-publisher`，避免重复发布 TF。

在线建图目标按环境分开保存：仿真目标位于 `online_slam_goals/simulation/`，实车目标位于 `online_slam_goals/reality/`。这些路径均相对于项目根目录，不依赖电脑用户名或仓库的安装位置。

开机后旁路录制 MID360 建图输入的脚本、systemd 安装脚本和用法说明都集中在 `autostart_mid360_record/`，详见 `autostart_mid360_record/README.md`。
-------------------------------------------------------------------------------
(7)通过离线重跑录制包来建图
[注意不要再启动真实 Livox driver]

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py slam:=True use_robot_state_pub:=True use_livox_driver:=False use_sim_time:=True
ros2 bag play <BAG_DIR> --clock
```

## 实车地图归位
想要将实车建图生成的地图导入实车先验导航，须进行以下操作：

`slam:=True` 负责建图，`slam:=False` 负责读取先验地图导航，二者按顺序分开启动。自动保存的 `auto_map_*.yaml/.pgm` 和 `point_lio/PCD/scans.pcd` 不会自动替换先验导航的 `game.*`。只保存当前目录的 `game.yaml/game.pgm` 也不够，必须完成下面的导入步骤。栅格地图与点云应来自同一次建图，不要把旧栅格地图与新点云混用。

1.建图完成后，先不要关闭建图程序，新开终端保存栅格地图：

```bash
cd /home/pfa/sight/new/pfa-nav-own
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -t /map -f "$PWD/game" --fmt pgm --mode trinary --ros-args -p save_map_timeout:=15.0 -p map_subscribe_transient_local:=true
```

`-f game` 保存到执行命令时的当前目录，不会直接导入先验导航目录。请等待 `Map saved successfully`，并检查当前目录中 `game.yaml` / `game.pgm` 的修改时间和文件大小，不能只凭旧文件仍然存在判断本次成功。保存失败时先确认在线建图终端没有退出，并运行 `ros2 topic info /map -v` 检查地图发布者；如果使用了 `--namespace robot1`，保存时相应改为 `-t /robot1/map`。两个终端应使用相同的 `ROS_DOMAIN_ID` 和 ROS 通信配置。没有地图发布时，增加超时也不能保存地图。

2.保存完成后再结束建图程序，等待日志出现 `Saved final 3D point cloud`，确认 point_lio 已写出 `src/pb2025_sentry_nav/point_lio/PCD/scans.pcd`。随后在工作空间根目录运行脚本，把实车地图和点云复制到导航默认读取的位置，保留原始输出：

```bash
bash tools/map/prepare_reality_map.sh game --copy
```

脚本会执行这些操作：

(1)- 将当前目录下的 `game.yaml` / `game.pgm` 复制到 `src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/`
(2).- 将 `src/pb2025_sentry_nav/point_lio/PCD/scans.pcd` 复制到 `src/pb2025_sentry_nav/pb2025_nav_bringup/pcd/reality/game.pcd`
(3).- 自动修改导入后的 `game.yaml` 文件，将其中的 `image` 字段改成 `game.pgm`

省略 `--copy` 时仍使用原来的移动模式。导入脚本会先检查三份源文件和目标冲突，避免只装入部分地图；导航启动前也会检查 YAML、地图图像和 PCD 是否齐全。

3.如果保存地图时使用了别的名字，例如 `my_map.yaml` / `my_map.pgm`，但导航时仍想用 `world:=game`：
就执行以下命令：

```bash
bash tools/map/prepare_reality_map.sh game --map-name my_map --copy
```

4.仓库自带的 `map/reality/game.yaml/game.pgm` 属于旧地图。首次替换以及以后重新建图时，若目标目录已有同名文件，先备份需要保留的旧文件，再执行以下命令覆盖：

```bash
bash tools/map/prepare_reality_map.sh game --copy --force
```

5.导入完成后无需重新编译，可以直接启动实车导航：

```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=game slam:=False use_robot_state_pub:=True
```

也可使用实车先验导航入口（默认不开 RViz；地图和点云都读取导入后的 `reality/game.*`，不会读取下一次建图生成的临时 `scans.pcd`）：

```bash
USE_RVIZ=True bash tools/match_start/reality/nav.sh
```

如果完整机器人系统已在发布上述 TF，直接启动时设 `use_robot_state_pub:=False`，脚本入口则设 `USE_ROBOT_STATE_PUB=False`。导航启动成功的标志是 `lifecycle_manager_navigation` 输出 `Managed nodes are active`；地图正常显示不代表导航已经激活。`Could not find .../pcd/reality/game.pcd` 表示尚未导入点云；持续出现 `base_footprint ... does not exist` / `gimbal_yaw_fake ... does not exist` 则先检查机器人 TF 发布及命名空间。

注意区分运行失败与退出异常：`Ctrl+C` 后 Livox/RViz 崩溃或组件容器退出超时属于另一类关闭问题，不能据此判断之前的建图没有成功。是否保存成功应检查 `Map saved successfully`、`Saved final 3D point cloud` 以及实际文件；这些关闭问题仍需单独排查。

## 在rviz可视化机器人模型

```bash
ros2 launch pb2025_robot_description robot_description_launch.py robot_name:=pfa_sentry_robot
```

> 如果因为libusb问题导致pointlio拉起失败，临时解决在命令前面加环境变量LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0

## 修改传感器位置

更改 src/pb2025_robot_description/resource/xmacro/simulation_robot.sdf.xmacro,如果是实车就是pfa_sentry_robot.sdf.xmacro中的雷达位置，然后根据/livox/imu话题输出更改nav2_params.yaml的gravity记得加上负号

## 设置航点，用于多点导航

进下面launch文件改目录

### 实车参数配置

改launch下的add_waypoint_reality.launch.py：

```bash
ros2 launch wp_map_tools add_waypoint_reality.launch.py yaml_name:=red.yaml
```

```bash
ros2 launch wp_map_tools add_waypoint_reality.launch.py yaml_name:=blue.yaml
```

也可以通过 map 参数指定地图（默认 pb2025_nav_bringup/map/reality/game.yaml）：

```bash
ros2 launch wp_map_tools add_waypoint_reality.launch.py map:=src/pb2025_sentry_nav/pb2025_nav_bringup/map/reality/your_map.yaml
```

### 仿真参数配置

```bash
ros2 launch wp_map_tools add_waypoint_simulation.launch.py
```

记得改game.py里的self.nav_ac = ActionClient(self, NavigateToPose, '/red_standard_robot1/navigate_to_pose')

## 保存航点

```bash
source install/setup.bash
```

(改目录 src/wp_saver.cpp)

蓝方血量/弹量导航的航点应保存到当前工作区的配置目录：

```bash
ros2 run wp_map_tools wp_saver -f /home/pfa/sight/new/pfa-nav-own/auto_nav/config/waypoints_blue.yaml
```

红方航点也应通过 `-f` 指定实际配置文件路径，避免写入旧工作区。

## LIO 调参指南（整理自各个 Issue）

- 室内可以把 filter_size_surf, filter_size_map 调小一点，一般分别为 0.05, 0.15. 对于 ouster 或者这种点特别多的，point_filter_num 可以调大，比如 5~10.
- 当点云较密集时，用较大的 lidar_meas_cov。结构较单一时，用较大的 lidar_meas_cov 。

## 补充

可使用gimp修剪栅格地图，CloudCompare修剪先验点云

src/pb2025_sentry_nav/pb2025_nav_bringup/config/reality/mid360_user_config.json  在这里配置雷达和本机ip可用LivoxViewer.sh查看

本项目引入 namespace 的设计，与 ROS 相关的 node, topic, action 等都加入了 namespace 前缀。如需查看 tf tree，请使用命令:

```bash
ros2 run rqt_tf_tree rqt_tf_tree --ros-args -r /tf:=tf -r /tf_static:=tf_static -r __ns:=/red_standard_robot1
```
