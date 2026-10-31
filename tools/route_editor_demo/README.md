# 独立路线编排 GUI

此工具与现有导航 GUI 独立，仅供离线编辑和动画预览。没有 ROS、SSH、网络、实际坐标、实车执行或外部动作接口。不读取或修改 online_slam_goals 中的数据。

## 在小电脑桌面终端启动

```bash
cd /home/pfa/sight/new/pfa-nav-own
python3 tools/route_editor_demo/route_editor.py
```

无需 source ROS 环境。需要 Python 3 和 tkinter；通过普通 SSH 启动时需自行提供图形显示环境。

## 使用

- 添加航点：输入名称，作为纯文本标签。
- 添加等待：输入秒数，等待与航点是同级步骤。
- 新步骤插在选中项后；没有选中项则追加。
- 双击或点“编辑”修改名称/等待时长。
- 上移、下移、删除适用于两类步骤。
- “示例路线”生成 a1 → a2 → a3 → a4 → 等待15秒 → a3 → a2 → a1。
- 保存/打开使用专属 offline_route_storyboard/v1 JSON 格式，不兼容实车路线文件。
- 模拟预览：航点固定模拟0.8秒，等待按设定秒数计时；支持暂停、继续、取消。
- 预览时禁止编辑；取消后不推进后续步骤。

## 验证

```bash
cd tools/route_editor_demo
python3 -m unittest -v test_route_editor
```

没有显示环境时 GUI 测试会明确跳过；可用 xvfb-run -a python3 -m unittest -v test_route_editor 运行虚拟显示测试。

## 范围

此目录为独立演示，不修改 auto_nav/online_slam_goal_gui.py、目标执行器、导航启动脚本、地图文件或比赛状态话题。未实现机器人动作适配、实车路线格式转换或自瞄耦合。