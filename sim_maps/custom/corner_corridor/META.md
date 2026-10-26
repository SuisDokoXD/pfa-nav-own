# corner_corridor

- 分类：custom
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/corner_corridor_world.sdf`
- 场地模型：无（world 内直接建模）
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/corner_corridor.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/corner_corridor.yaml`
  - 尺寸：400×224 px @ 0.05 m → 20.00 × 11.20 m
  - origin：[-10.0, -2.0, 0]
  - mode：trinary
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：(-6.0, 0.0, 0.28) yaw=0.0

## 启动

```bash
./sim_maps/use_map.sh corner_corridor
```
