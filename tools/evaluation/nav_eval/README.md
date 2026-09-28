# Nav2 导航评分工具

这个工具用于评估一次 Nav2 导航任务的实际效果，支持实时评分和 rosbag2 录包离线评分。结果以 JSON 输出，分数范围为 `0~100`，同时包含各项可解释指标。

## 实时评分

仿真环境默认使用 namespace `red_standard_robot1`，并自动使用仿真时钟：

```bash
cd ~/src/sight/pfa-nav
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/evaluation/nav_eval/nav_score.py score_live \
  --environment simulation \
  --goal-x 3.2 \
  --goal-y -1.4 \
  --goal-frame map
```

实车环境默认使用空 namespace，并使用系统时间：

```bash
python3 tools/evaluation/nav_eval/nav_score.py score_live \
  --environment reality \
  --goal-x 3.2 \
  --goal-y -1.4 \
  --goal-frame odom
```

如果 namespace 不同，可以使用 `--namespace` 覆盖：

```bash
python3 tools/evaluation/nav_eval/nav_score.py score_live \
  --environment simulation \
  --namespace my_robot \
  --goal-x 3.2 \
  --goal-y -1.4 \
  --goal-frame map
```

实时模式会一直运行。导航结束后按 `Ctrl+C`，终端会输出最终评分。添加 `--live-report` 后，每秒输出一次当前评分快照。

## 录包离线评分

先查看录包中的话题：

```bash
ros2 bag info /path/to/bag
```

然后执行评分：

```bash
python3 tools/evaluation/nav_eval/nav_score.py score_bag /path/to/bag \
  --environment simulation \
  --goal-x 3.2 \
  --goal-y -1.4 \
  --goal-frame map
```

如果录包使用的 namespace 或话题名称不同，可以使用 `--namespace` 或各个话题参数覆盖默认值。

## GUI 联动

启动给点 GUI 时，可以通过 `--environment` 选择环境：

```bash
# 仿真：默认 namespace 为 red_standard_robot1，默认目标目录为 simulation
python3 auto_nav/online_slam_goal_gui.py --environment simulation

# 实车：默认 namespace 为空，默认目标目录为 reality
python3 auto_nav/online_slam_goal_gui.py --environment reality
```

GUI 中勾选“启用本次导航评分”后再启动目标，GUI 会启动评分器并自动打开“导航评分可视化”窗口。窗口会每 0.5 秒刷新一次，显示总分、成功状态、真值闭环、路径跟踪、安全距离、激光稳定性/噪点、路径平滑度、规划稳定性等分项，以及跟踪误差、障碍距离、尖峰率和扫描抖动等原始观测值。

也可以先点击“打开评分可视化面板”查看空面板；这个按钮不会启动导航或评分。没有勾选评分时，GUI 的行为与原来的给点/巡航功能一致。

路线测试同样使用这个面板。当前面板对应一整次 GUI 启动的导航任务；如果执行多航点路线，最终分数由评分器在路线结束时汇总。调参时建议保存每次面板对应的最终 JSON，而不是只记录一个肉眼看到的总分。

GUI 结束一次评分后会自动把最终结果保存到当前目标目录的 `scores/` 子目录：仿真默认是 `online_slam_goals/simulation/scores/`，实车默认是 `online_slam_goals/reality/scores/`。这些 JSON 可以直接作为调参器的输入或后处理数据。

面板中的“质量估计分”是实时曲线：它只反映当前已经采集到的质量项，不把“任务尚未结束”当作成功；最终 `score` 仍然会乘上导航成功状态。因此导航过程中可能看到质量估计分，而最终总分要等 action 成功/失败后才确定。

## 评分内容

评分会综合以下内容：

- Nav2 action 的成功、取消、失败和超时状态；
- 最终位置误差和导航耗时；
- 全局路径 `/plan` 的长度和重规划次数；
- 局部控制路径 `/local_plan` 的长度和重规划次数；
- 机器人相对局部路径的跟踪误差；
- 激光扫描得到的最小障碍物距离；
- 激光有效回波率、孤立尖峰率和静止时的帧间抖动；
- Nav2 输出指令与底盘实际指令之间的速度误差；
- 指令延迟和控制平滑性；
- `map -> gimbal_yaw_fake` 的 TF 可用性；
- 目标、里程计和规划路径之间的坐标系一致性。

在仿真模式下还会读取 Gazebo 提供的真值里程计
`/chassis_odometry_gt`，计算定位位置误差、航向误差和速度误差。仿真综合分会加入这部分真值指标；实车模式没有真值话题，不会因为缺少该话题扣分。仿真真值话题由机器人 namespace 自动补全，例如：

```text
/red_standard_robot1/chassis_odometry_gt
```

评分器还会统计激光有效回波率、孤立尖峰率、静止或低速时的帧间抖动，以及局部路径的航向变化，用于判断传感器输出是否稳定、路径是否出现频繁折线。孤立尖峰率不是最终的几何噪声真值：真实的细障碍也可能形成尖峰。更严格的第二阶段应使用仿真地图和真值位姿对每束激光做射线预测，再计算实际距离与理想距离的残差。

评分时会去掉仿真桥接器加在 `frame_id` 前面的机器人 namespace，例如把 `red_standard_robot1/odom` 与 `odom` 视为同一坐标系。实时模式还会尝试用 TF 把 `gimbal_yaw_fake` 下的局部路径变换到里程计坐标系后再计算跟踪误差。局部路径的周期性重新发布不再直接算作重规划，只有形状或长度发生明显变化才计数。激光帧间抖动只在机器人平移和角速度都很低时统计，原地转向不会被误判为传感器噪声。

缺失的数据会显示为 `null`，并按零分参与综合评分，不会被当作理想结果。

## 本仓库中的话题和坐标系

| 数据 | 话题 | 说明 |
|---|---|---|
| 全局规划路径 | `/plan` | 通常使用 `map` 坐标系 |
| 局部控制路径 | `/local_plan` | 通常使用 `odom` 坐标系 |
| Nav2 控制指令 | `/cmd_vel_nav2_result` | 控制器输出 |
| 底盘控制指令 | `/cmd_vel` | 经过 `fake_vel_transform` 后的指令 |
| 位姿和速度 | `/odometry` | 坐标系以消息中的 `header.frame_id` 为准 |
| Nav2 控制基座 | `gimbal_yaw_fake` | Nav2 costmap 和控制器使用的基座 |
| 导航 action 状态 | `/navigate_to_pose/_action/status` | 判断任务最终状态 |
| 仿真真值里程计 | `/chassis_odometry_gt` | 仅仿真可用，由 Gazebo bridge 提供 |

评分器会检查消息中的 `header.frame_id`。如果路径坐标系和位姿坐标系不一致，相关跟踪误差不会直接计算，而会记录 `frame_mismatch_count`。

## 主要参数

```text
--environment simulation|reality  选择仿真或实车环境
--namespace NAME                   覆盖环境默认 namespace
--ground-truth-odom-topic TOPIC    仿真真值里程计，默认 /chassis_odometry_gt
--goal-x X --goal-y Y              目标位置
--goal-frame map|odom              目标坐标系
--goal-tolerance DIST              到达目标的位置容差，默认 0.25 m
--timeout SECONDS                  任务最大时间，默认 300 s
--live-report                      实时模式下每秒输出评分快照
--result-file FILE                 退出时保存最终 JSON 结果
--live-report-file FILE             将最新实时快照写入 JSON，供 GUI 或其他程序读取
```

## 仿真闭环调参接口

`sim_tuner.py` 使用 SPSA 生成参数实验，不直接假设 Gazebo 或 GUI 的启动方式。每轮会生成一组 `plus` 和 `minus` 参数文件；外部仿真运行器分别执行固定路线并把评分 JSON 写回，随后 `tell` 根据两次结果估计梯度并生成下一组参数。

先创建调参状态：

```bash
python3 tools/evaluation/nav_eval/sim_tuner.py init \
  --base-params src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params.yaml \
  --space tools/evaluation/nav_eval/simulation_tuning_space.yaml \
  --state /tmp/nav_tuning_state.json \
  --output-dir /tmp/nav_tuning_trials \
  --routes route_a route_b route_c \
  --repeats 3
```

生成一轮实验：

```bash
python3 tools/evaluation/nav_eval/sim_tuner.py propose \
  --state /tmp/nav_tuning_state.json
```

命令会输出两个 trial 请求，并生成：

```text
/tmp/nav_tuning_trials/iteration_0000/plus_params.yaml
/tmp/nav_tuning_trials/iteration_0000/minus_params.yaml
```

外部运行器使用请求中的两个参数文件，分别运行 `routes` 中的固定路线，每条路线重复 `repeats` 次，并使用对应的 `seed`。运行器将 9 次结果聚合成一个评分 JSON，保存为对应的 `plus_result.json` 和 `minus_result.json`。完成后更新参数：

```bash
python3 tools/evaluation/nav_eval/sim_tuner.py tell \
  --state /tmp/nav_tuning_state.json \
  --plus-result /tmp/nav_tuning_trials/iteration_0000/plus_result.json \
  --minus-result /tmp/nav_tuning_trials/iteration_0000/minus_result.json
```

这是第一版闭环接口：评分器负责测量，`sim_tuner.py` 负责提出参数和更新方向，仿真启动器负责执行实验。后续再把固定路线 GUI 或自动仿真启动脚本接到 trial 请求上。
