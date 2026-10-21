# pb_rm_interfaces

ROS 2 接口定义（`.msg`），供本仓库的导航、仿真与裁判系统数据解析使用。

源自 PolarBear StandardRobot++ 的接口包，此处为随工作区一起维护的副本，**以本目录下的实际文件为准**。

## msg

| 文件 | 说明 |
|---|---|
| `Gimbal.msg` | 云台姿态：`pitch`、`yaw`；另有 `pitch_min_range` / `pitch_max_range` / `yaw_min_range` / `yaw_max_range` 四个限幅字段，只在速度控制模式使用 |
| `GimbalCmd.msg` | 云台控制命令，带 `std_msgs/Header`。支持两种控制类型：`ABSOLUTE_ANGLE = 1`（绝对角度）与 `VELOCITY = 2`（速度），由 `yaw_type` / `pitch_type` 分别指定，数据分别放在 `position` / `velocity` 字段 |
| `Models.msg` | 机器人各部位型号字符串：`chassis`、`gimbal`、`shoot`、`arm`、`custom_controller` |
| `RobotStateInfo.msg` | `std_msgs/Header` + `Models` |

## msg/referee

对应 RoboMaster 裁判系统串口协议 **V1.7.0（20241225）**。

| 文件 | 协议包 | 说明 |
|---|---|---|
| `GameStatus.msg` | 0x0001 | 比赛状态：`game_progress`（`NOT_START` / `PREPARATION` / `SELF_CHECKING` / `COUNT_DOWN` / `RUNNING` / `GAME_OVER`）与 `stage_remain_time` |
| `GameRobotHP.msg` | 0x0003 | 双方机器人、前哨站、基地血量 |
| `EventData.msg` | 0x0101 | 场地事件：补给区、大小能量机关、中央/梯形高地、中心增益点 |
| `RobotStatus.msg` | 0x0201 / 0x0202 / 0x0203 / 0x0206 / 0x0208 | 机器人性能体系：ID 与等级、血量与枪口热量、本机位姿、扣血原因、允许发弹量与剩余金币 |
| `Buff.msg` | 0x0204 | 机器人增益（回血/冷却/防御/负防御/攻击）与剩余能量 |
| `RfidStatus.msg` | 0x0209 | 24 个 RFID 增益点的触发状态（字段均为 `bool`） |
| `GroundRobotPosition.msg` | 0x020B | 己方英雄、工程、3 号步兵、4 号步兵的位置 |

## 不在本包、但常一起使用的消息

| 用途 | 消息 |
|---|---|
| 底盘控制 | `rmoss_interfaces/msg/ChassisCmd`（内部含 `geometry_msgs/Twist`） |
| 射击命令 | `rmoss_interfaces/msg/ShootCmd` |

## 已知说明

- `RfidStatus.msg` 里定义了 `NOT_DETECTED = 0` / `DETECTED = 1` 两个常量，但字段全部是 `bool`，这两个常量目前没有被使用。
- `RobotStatus.msg` 只保留了 `0x0202` 中的 `shooter_17mm_1_barrel_heat`，未包含 `buffer_energy`（底盘缓冲能量）和 `shooter_17mm_2_barrel_heat`。若需要底盘缓冲能量，需要补字段。
