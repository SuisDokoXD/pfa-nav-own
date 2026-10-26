# rmul_2024

- 分类：rmul
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/rmul_2024_world.sdf`
- 场地模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmul_2024`
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmul_2024.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmul_2024.yaml`
  - 尺寸：272×210 px @ 0.05 m → 13.60 × 10.50 m
  - origin：[-3.75, -4.54, 0]
  - mode：trinary
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：(4.3, 3.35, 1.2) yaw=0.0

## 启动

```bash
./sim_maps/use_map.sh rmul_2024
```
