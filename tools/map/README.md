# 离线标点（把目标点绑定到一张地图）

用途：**建图完成后，用最终版地图标点**，供之后"导入同一张图"的先验导航使用。
不需要实车、不需要 LIO、不需要 Nav2。

## 为什么必须绑定 map_id

目标点 JSON 里记录 `frame`：

- `frame: "map"` —— 会写入 `map.map_id`（对 `/map` 的宽高/分辨率/origin/栅格做 sha256）。
  先验导航加载**同一张图**时校验通过；换成别的图、或对地图做过调平/改 origin，`map_id` 变化，工具会**拒绝加载**（这是保护，不是故障）。
- `frame: "odom"` —— 临时点，只保存 JSON、不导出 auto_nav 航点 YAML，**仅当前会话有效**。

> 注意：建图过程中 `/map` 每 0.2 s 刷新，`map_id` 会随地图变化。**不要在建图途中录正式点**，
> 建图结束、保存地图、确认不再修改地图之后再标点。

## 操作流程

```bash
# 1) 建图（slam 模式）；无裁判系统时加 --test-mode
./auto_nav/start_reality_online_slam_nav.sh --test-mode

# 2) 走完场地后保存地图
ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: map, map_url: /绝对路径/site, image_format: pgm}"
#    同时确认 point_lio 的 PCD（scans.pcd）也已保存

# 3) 若需要调平/改原点：在这一步之前用 CloudCompare 处理完，之后再标点

# 4) 用最终地图标点（只起 map_server + 静态 TF）
tools/map/annotate_map.sh --map /绝对路径/site.yaml --rviz

# 5) 另开终端打开录点界面（默认已是 map 帧）
python3 auto_nav/online_slam_goal_gui.py --record-frame map
```

## 标点时的三条硬约束

1. **RViz 的 Fixed Frame 必须是 `map`**。`online_slam_goal.py` 会校验点击点的 `header.frame_id`，
   Frame 不对会被拒绝录点。
2. **拿不到 `/map` 时拒绝保存**：录点脚本现在不再写 `map: null` 的坏文件，而是报错退出。
3. **地图一旦修改，旧点作废**：调平、改 origin、替换图片都会改变 `map_id`。

## 验收（建议每次换图都做一遍）

1. `annotate_map.sh` + GUI 记一个点；
2. 关闭后按**先验图定位模式**启动，加载同一张 `site.yaml`；
3. GUI 能打开该点、不报 `map_id` 不一致 → 通过；
4. 实车发一次目标，确认到点。

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `ros2 node list` 出现重复 `/map_server`，随后 `lifecycle get` 报 `Node not found` | ros2 daemon 缓存了已退出的节点 | 脚本启动时会 `ros2 daemon stop` 清缓存；也可手动执行 |
| 自己写的订阅端收不到 `/map` | `/map` 是 `transient_local`（latched） | 订阅必须设置 `durability=TRANSIENT_LOCAL` |
| 点击后 GUI 报"本次点击已拒绝" | RViz Fixed Frame 不是 `map` | 改 Fixed Frame |
| 已有 `/map_server` 在跑 | 脚本检测到会直接退出（exit 4） | 先结束旧节点 |
