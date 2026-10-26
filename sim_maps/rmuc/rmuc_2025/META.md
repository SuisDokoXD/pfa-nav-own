# rmuc_2025

- 分类：rmuc
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/rmuc_2025_world.sdf`
- 场地模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2025`
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2025.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2025.yaml`
  - 尺寸：583×324 px @ 0.05 m → 29.15 × 16.20 m
  - origin：[-3.58, -9.44, 0]
  - mode：trinary
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：(4.4, 8.0, 0.28) yaw=0.0

## 启动

```bash
./sim_maps/use_map.sh rmuc_2025
```
