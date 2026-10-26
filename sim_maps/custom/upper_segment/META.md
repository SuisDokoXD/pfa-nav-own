# upper_segment

- 分类：custom
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/upper_segment_world.sdf`
- 场地模型：无（world 内直接建模）
- 地图：`src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/upper_segment.pgm` + `src/pb2025_sentry_nav/pb2025_nav_bringup/map/simulation/upper_segment.yaml`
  - 尺寸：260×91 px @ 0.05 m → 13.00 × 4.55 m
  - origin：[-6.5, -0.3, 0.0]
  - mode：trinary
- nav2 参数：`src/pb2025_sentry_nav/pb2025_nav_bringup/config/simulation/nav2_params_upper_segment.yaml`　← 专用文件，切这个场地要显式传参
- 出生点：(5.55, 1.89, 0.10) yaw=3.1415926

## 启动

```bash
./sim_maps/use_map.sh upper_segment
```
