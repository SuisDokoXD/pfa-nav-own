# empty

- 分类：custom
- 源 world：`src/rmu_gazebo_simulator/rmu_gazebo_simulator/resource/worlds/empty_world.sdf`
- 场地模型：无（world 内直接建模）
- 地图：**无**（该场地没有配 pgm/yaml）
- nav2 参数：`config/simulation/nav2_params.yaml`（共享）
- 出生点：`gz_world.yaml` 里没有该场地条目

## 启动

```bash
./sim_maps/use_map.sh empty
```
