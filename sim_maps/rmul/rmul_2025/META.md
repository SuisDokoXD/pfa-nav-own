# rmul_2025

- 分类：rmul
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/rmul_2025_world.sdf`
- 场地模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmul_2025`
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmul_2025.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmul_2025.yaml`
  - 尺寸：242×162 px @ 0.05 m → 12.10 × 8.10 m
  - origin：[-1.25, -7.55, 0]
  - mode：trinary
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：(-4.8, 3.5, 0.28) yaw=0.0

## 启动

```bash
./sim_maps/use_map.sh rmul_2025
```
