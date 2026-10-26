# rmuc_2027

- 分类：rmuc
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/rmuc_2027_world.sdf`
- 场地模型：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/models/rmuc_2027`
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2027.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/rmuc_2027.yaml`
  - 尺寸：266×203 px @ 0.08 m → 21.28 × 16.24 m
  - origin：[-4.4, -8.13, 0]
  - mode：trinary
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：(-9.69, 0.0, 0.38) yaw=0.0

## 启动

```bash
./sim_maps/use_map.sh rmuc_2027
```
