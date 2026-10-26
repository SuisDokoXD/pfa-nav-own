# upper_segment 运行说明（v4）

只含 CAD 图纸**上半段**（原图 `v=73..336` → 世界 `y ∈ [0.20, 3.75]`）的 Gazebo 场地。
生成器 `tools/sim/drawing_to_upper_segment.py`；校验器 `tools/sim/verify_upper_segment.py`。

## 1. 图纸语义（按用户口径确认）

| 图纸元素 | 含义 | 处理 |
|---|---|---|
| 深紫 (121,121,171) | 赛道 / 可行驶地面 | free |
| 浅紫 (168,168,196) | 墙体本体 / 场外非行驶区 | occupied（0.60 m 高墙） |
| 蓝线（1–2 px） | 边界描边 | 不单独膨胀（膨胀会在赛道里切出假墙） |
| 灰色块 | **障碍物** | 不做实体块；沿它的边界做 **0.10 m 厚、0.45 m 高** 的围栏薄墙，围内记 unknown |
| 青（灰台下） | **起伏路** | **地形不建**，同样用围栏薄墙围起来 |
| 青（左侧） | 免打区 | 地面涂装（free + 地贴） |
| 红 | 启动区 | **也是赛道**，只是出生点在那里（free + 地贴） |
| 绿 | 目标带 | 地面涂装（free + 地贴） |

围栏墙高 0.45 m，取「与机器人整体高度相近或略高」；赛道边界墙仍为 0.60 m（高于雷达安装面）。

前三版走过的弯路，留档防止再犯：

| 版本 | 语义 | 结果 |
|---|---|---|
| v1 | 深紫 = 障碍 | 赛道整片变墙，场内 occupied 66%、free 36%，不可用 |
| v2 | 浅紫细带 = 墙，其余全自由 | 场地被切成碎片，按底盘半宽 0.19 m 腐蚀后只有 6.9% 可达 |
| v3 | 深紫 = 赛道、浅紫 = 墙（灰当作 0.10/0.20 台阶） | 台阶被埋进 0.60 m 实心块，与图纸「障碍物」不符 |
| v4 | 同上，但灰/青改为**围栏薄墙**，起伏路不建地形 | 场内 occupied 42.8% / free 49.5%，单一连通、可达约 48% |

## 2. 生成物

| 文件 | 内容 |
|---|---|
| `.../resource/worlds/upper_segment_world.sdf` | 4 个静态模型：`arena_ground_upper` / `upper_segment_geometry`（80 面赛道墙 + 8 根围栏，共 88 个 box）/ `upper_segment_boundary`（4 面围墙，互相重叠、四角无缝）/ `upper_segment_markings`（启动区 / 目标带 / 免打区 3 块地贴）；`<physics>` 0.01 s / 100 Hz |
| `.../map/simulation/upper_segment.{pgm,yaml}` | 260×91 @ 0.05 m，`origin [-6.500, -0.300]`；全图 occupied 42.9% / free 35.8% / unknown 21.3%；**场内** occupied 42.8% / free 49.5% |
| `.../config/simulation/nav2_params_upper_segment.yaml` | 按实测车体写 `footprint`（0.47×0.37 m 外接矩形）取代 `robot_radius: 0.3`；`inflation_radius` 0.7→0.30、`cost_scaling_factor` 4.0→3.0 |
| `tools/sim/drawings/upper_segment_preview.png` | 红=碰撞掩码、黄框=障碍物、青框=起伏路、白点=出生点 |

围内（障碍物内部、起伏路内部）在地图里是 **unknown(127)** 而不是 occupied：不可行驶，但也不假装是墙。

## 3. 怎么跑

```bash
cd ~/sight/pfa-nav
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=upper_segment

ros2 launch pb2025_sentry_nav rm_navigation_simulation_launch.py \
    world:=upper_segment use_rviz:=True \
    params_file:=$PWD/src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params_upper_segment.yaml
```

出生点 `(5.55, 1.89, 0.10)`、`yaw = π`（朝赛道），写在 `config/gz_world.yaml` 的 `upper_segment` 段。

## 4. 重新生成与自检

```bash
python3 tools/sim/drawing_to_upper_segment.py \
    --drawing tools/sim/drawings/rmu_arena_2026.png \
    --out-world src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds \
    --out-map  src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation \
    --preview  tools/sim/drawings/upper_segment_preview.png \
    --report   /tmp/upper_report.json

python3 tools/sim/verify_upper_segment.py \
    --world src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/upper_segment_world.sdf \
    --map   src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/upper_segment.yaml \
    --report /tmp/upper_report.json --self-test
```

硬断言（不达标 exit 1）：

- 场内自由率 ≥ 40%、占据率 35–60%（实测 49.5% / 42.8%）
- 起始门洞净宽 ≥ 0.50 m（实测 0.95 m）
- 底盘半宽 0.25 m 腐蚀后：**单一连通域**、可达面积 ≥ 8 m²（实测约 10 m²）
- 出生点 → 场内最开阔点最小净空 ≥ 0.25 m（实测 0.40 m，通道 0.80 m）
- 出生点最近障碍 ≥ 0.30 m（实测 0.45 m）
- 每组围栏：4 根都在占据格、围内为 unknown、围内不可行驶
- `--self-test`：origin 平移一格必须判负；删掉一个墙盒必须出现覆盖空洞

`min_corridor=0.50` 的依据是实测车体：底盘 0.36×0.24 m，轮组外沿 0.47×0.37 m。

## 5. 已实测（pfa-mini-pc，Ignition Gazebo 6.17.1）

- `ign sdf -k` → `Valid.`
- `verify_upper_segment.py --self-test` → 31 项全过
- headless `ign gazebo -s -r --iterations 400` → rc=0、0 error
- `bringup_sim.launch.py world:=upper_segment`：4 个场地模型 + 机器人加载；
  `/world/default/pose/info` 显示落点 `x=5.55 y=1.89 z=0.10`、`yaw≈π`，未被弹开

## 6. 已知限制

- **本机没有可用 GPU 渲染**：`ign gazebo` 的 GUI 必须 `xvfb-run` 才能起；即便如此，
  render 类传感器（`rplidar_a2/scan`、`livox/lidar`）只注册话题、不出数据，
  EGL 报 `Not allowed to force software rendering when API explicitly selects a hardware device`。
  地图与几何不受影响，但雷达避障验收需要在有 GPU/驱动的机器上复验。
- 起伏路地形按用户要求不建模，只围起来；若以后要做起伏，需要单独加 terrain 模型。
- 左侧靶标区在蓝墙之外，属场外，不可行驶。

## 7. 回滚

```bash
cp -a /tmp/upper_bak_*/<相对路径> ~/sight/pfa-nav/
```

world 在 `install/` 下是符号链接，改 world / map / 参数都不需要重编。
