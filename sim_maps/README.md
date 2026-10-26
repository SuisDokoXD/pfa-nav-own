# 仿真地图汇总（sim_maps）

`pfa-nav-own` 里所有可跑的 Gazebo 仿真场地都汇总在这里，按赛制分类。
**原位置的文件没有移动**，这里全部是副本，方便一眼看清有哪些图、怎么切。

当前默认场地：`rmuc_2027`

## 一览

| 场地 | 分类 | 地图 | 分辨率 | 地图范围 | 出生点 | 参数 |
|---|---|---|---|---|---|---|
| `corner_corridor` | custom | 有 | 0.05 | 20.0×11.2 m | (-6.0, 0.0) | 共享 |
| `rmuc_2024` | rmuc | 有 | 0.05 | 28.9×15.1 m | (6.35, 7.6) | 共享 |
| `rmuc_2025` | rmuc | 有 | 0.05 | 29.2×16.2 m | (4.4, 8.0) | 共享 |
| `rmuc_2026` | rmuc | 有 | 0.05 | 28.2×15.3 m | (4.4, 8.0) | 共享 |
| `rmuc_2027` | rmuc | 有 | 0.08 | 21.3×16.2 m | (-9.69, 0.0) | 共享 |
| `rmul_2024` | rmul | 有 | 0.05 | 13.6×10.5 m | (4.3, 3.35) | 共享 |
| `rmul_2025` | rmul | 有 | 0.05 | 12.1×8.1 m | (-4.8, 3.5) | 共享 |
| `rmul_2026` | rmul | **无** | — | — | (1.5, 3.0) | 共享 |
| `upper_segment` | custom | 有 | 0.05 | 13.0×4.5 m | (5.55, 1.89) | 专用 |

## 切场地

```bash
./sim_maps/use_map.sh                    # 列出所有场地
./sim_maps/use_map.sh rmuc_2026           # 打印该场地的启动命令
./sim_maps/use_map.sh rmuc_2026 -x        # 直接执行（两个终端要分开放，建议先不加 -x）
```

底层其实就是两个 launch：

```bash
ros2 launch rmu_gazebo_simulator bringup_sim.launch.py world:=<名字>
ros2 launch pb2025_sentry_nav rm_navigation_simulation_launch.py world:=<名字> use_rviz:=True \
    params_file:=<按 INDEX.yaml 决定的参数文件>
```

## 注意

- `world:=<名字>` 会同时决定 world 文件、地图文件名、`gz_world.yaml` 里的出生点 —— **三者必须同名**。
- 只有 `upper_segment` 有专用 nav2 参数；其余用共享 `config/simulation/nav2_params.yaml`。
- `rmuc_2027` 的地图 `rmuc_2027.pgm/.yaml` 目前**没有进 git**（本地未跟踪），这里的副本可用但仓库里缺源文件。
