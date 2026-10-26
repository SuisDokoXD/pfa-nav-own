# rmuc_2024

- 分类：rmuc
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/rmuc_2024_world.sdf`
- 场地模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2024`
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2024.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2024.yaml`
  - 尺寸：577×301 px @ 0.05 m → 28.85 × 15.05 m
  - origin：[-6.19, -7.48, 0]
  - mode：trinary
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：(6.35, 7.6, 0.2) yaw=0.0

## 启动

```bash
./sim_maps/use_map.sh rmuc_2024
```
