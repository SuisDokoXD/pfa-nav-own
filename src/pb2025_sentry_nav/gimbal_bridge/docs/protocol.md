# C 板串口协议对照表（过洞扩展版）

> 本文档对应 `gimbal_bridge` 包里的 `include/gimbal_bridge/sp_protocol.hpp`。
> 与湖南大学原版 `master_process.h` 相比，**新增了升降/隧道字段，CRC 偏移也变了，属于不兼容改动**。
> C 板固件与远端自瞄程序必须按本表同步修改。

---

## 1. 物理层

| 项 | 值 |
|---|---|
| 接口 | USB CDC ACM（`/dev/ttyACM0`） |
| 波特率 | 115200（虚拟，实际速率由 USB 决定） |
| 数据格式 | 8N1，无流控 |
| 字节序 | **小端**（STM32 与 x86 一致） |
| float | IEEE 754 单精度，4 字节 |
| 结构体 | `#pragma pack(1)`，无填充 |

两个方向的帧长**不同**，必须按各自的 `sizeof` 收发：

- 上位机 → C 板：**51 字节**
- C 板 → 上位机：**61 字节**

---

## 2. 上位机 → C 板：`VisionToGimbal`（51 字节）

| 偏移 | 字段 | 类型 | 大小 | 说明 |
|---:|---|---|---:|---|
| 0 | `head[0]` | uint8 | 1 | 固定 `'S'` = 0x53 |
| 1 | `head[1]` | uint8 | 1 | 固定 `'P'` = 0x50 |
| 2 | `mode` | uint8 | 1 | 0=不控制 1=控云台 2=控云台+开火 |
| 3 | `yaw` | float32 | 4 | 云台偏航目标角，弧度 |
| 7 | `yaw_vel` | float32 | 4 | 偏航角速度前馈 |
| 11 | `yaw_acc` | float32 | 4 | 偏航角加速度前馈 |
| 15 | `pitch` | float32 | 4 | 云台俯仰目标角，弧度 |
| 19 | `pitch_vel` | float32 | 4 | 俯仰角速度前馈 |
| 23 | `pitch_acc` | float32 | 4 | 俯仰角加速度前馈 |
| 27 | `vx` | float32 | 4 | 底盘前进速度，m/s |
| 31 | `vy` | float32 | 4 | 底盘左向速度，m/s |
| 35 | `angle_diff` | float32 | 4 | 目标与云台当前朝向的角差，弧度 |
| **39** | **`vw`** | float32 | 4 | **新增** 底盘旋转速度，rad/s |
| **43** | **`desire_lifter_pos`** | uint8 | 1 | **新增** 升降目标：0=顶 1=底 2=中，**255=保持不变** |
| **44** | **`tunnel_align_active`** | uint8 | 1 | **新增** 请求底盘航向对齐，1=请求 |
| **45** | **`tunnel_align_angle_deg`** | float32 | 4 | **新增** 对齐目标角，度 |
| **49** | `crc16` | uint16 | 2 | **偏移变更**（原 39）前 49 字节的 CRC |
| | | | **51** | |

`mode` 语义保持不变：

- `0`：不接管云台，`yaw`/`pitch`/`vx`/`vy` 全部应被忽略
- `1`：接管云台，不开火
- `2`：接管云台并允许开火

**安全约定**：`desire_lifter_pos = 255` 表示"不要动升降"，上电与急停时应保持该值。

---

## 3. C 板 → 上位机：`GimbalToVision`（61 字节）

| 偏移 | 字段 | 类型 | 大小 | 说明 |
|---:|---|---|---:|---|
| 0 | `head[0]` | uint8 | 1 | `'S'` |
| 1 | `head[1]` | uint8 | 1 | `'P'` |
| 2 | `mode` | uint8 | 1 | C 板当前模式 |
| 3 | `q[4]` | float32×4 | 16 | 云台姿态四元数，顺序 **w x y z** |
| 19 | `yaw` | float32 | 4 | 云台偏航角，弧度 |
| 23 | `yaw_vel` | float32 | 4 | 偏航角速度 |
| 27 | `pitch` | float32 | 4 | 云台俯仰角，弧度 |
| 31 | `pitch_vel` | float32 | 4 | 俯仰角速度 |
| 35 | `bullet_speed` | float32 | 4 | 弹速，m/s |
| 39 | `bullet_count` | uint16 | 2 | 累计发弹数 |
| 41 | `projectile_allowance` | uint16 | 2 | 剩余允许发弹量 |
| 43 | `current_hp` | uint16 | 2 | 当前血量 |
| 45 | `current_outpost_hp` | uint16 | 2 | 前哨站血量 |
| 47 | `current_base_hp` | uint16 | 2 | 基地血量 |
| 49 | `game_state` | uint8 | 1 | 0未开始 1准备 2自检 3倒计时 4比赛中 5结算 |
| 50 | `stage_remain_time` | uint16 | 2 | 当前阶段剩余，秒 |
| **52** | **`lifter_current_pos`** | uint8 | 1 | **新增** 当前档位：0=顶 1=底 2=中 |
| **53** | **`is_transformable`** | uint8 | 1 | **新增** 1=可变形；0=正在变形或卡住 |
| **54** | **`tunnel_yaw_aligned`** | uint8 | 1 | **新增** 1=航向对齐完成 |
| **55** | **`transform_state`** | float32 | 4 | **新增** 变形进度，0~100（百分比） |
| **59** | `crc16` | uint16 | 2 | **偏移变更**（原 52）前 59 字节的 CRC |
| | | | **61** | |

`q[4]` 顺序是 **w x y z**。由四元数求偏航角：

```c
yaw = 2.0f * atan2f(q[3], q[0]);   // q[3]=z, q[0]=w
```

---

## 4. CRC16

多项式 `0x1021`（反射形式 `0x8408`），初值 `0xFFFF`，**无最终取反**。

```c
uint16_t crc16(const uint8_t * data, uint32_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *data++;
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 1U) ? (uint16_t)((crc >> 1) ^ 0x8408U)
                             : (uint16_t)(crc >> 1);
        }
    }
    return crc;
}
```

**计算范围**：整帧去掉末尾 2 字节 CRC。

**存放顺序**：小端，`buf[N-2]` 是低字节，`buf[N-1]` 是高字节。

校验：

```c
uint16_t got = (uint16_t)(buf[N-2] | (buf[N-1] << 8));
if (crc16(buf, N - 2) != got) { /* 丢弃该帧 */ }
```

参考测试向量（可用于验证移植是否正确）：

| 输入 | 输出 |
|---|---|
| 空（长度 0） | `0xFFFF` |
| `"S"` | `0x6F99` |
| `"SP"` | `0x5BA2` |
| `"SP"` + 37 个 0x00（共 39 字节） | `0x3FFA` |

---

## 5. 与旧版的差异（改固件时重点看这里）

| 项 | 旧版 | 新版 | 影响 |
|---|---|---|---|
| `VisionToGimbal` 大小 | 41 | **51** | 收包长度为 51 才算完整帧 |
| `GimbalToVision` 大小 | 54 | **61** | 发包长度为 61 |
| `VisionToGimbal.crc16` 偏移 | 39 | **49** | CRC 位置变了，不兼容 |
| `GimbalToVision.crc16` 偏移 | 52 | **59** | 同上 |
| 前 39 / 52 字节 | — | **完全不变** | 原有字段偏移没动 |

**只算旧版 41 / 54 字节的固件会一帧都收不到**，因为长度和 CRC 位置都对不上。

---

## 6. C 板固件移植清单

1. 把 `VisionToGimbal_s` 结构体按第 2 节扩到 51 字节，`crc16` 放到末尾
2. 把 `GimbalToVision_s` 按第 3 节扩到 61 字节，`crc16` 放到末尾
3. 收包状态机：找 `'S' 'P'` → 等满 **51** 字节 → 校验 CRC → 解析
4. 发包：填满 61 字节 → 计算 CRC → 写入 `[59],[60]`（小端）
5. 实现三个新输入：
   - `vw` → 底盘角速度（如果底盘做全向解算）
   - `desire_lifter_pos` → 升降目标，**收到 255 时保持不动**
   - `tunnel_align_active` / `tunnel_align_angle_deg` → 航向对齐闭环
6. 回填四个新输出：
   - `lifter_current_pos`：当前档位
   - `is_transformable`：卡住或运动中填 0
   - `tunnel_yaw_aligned`：对齐完成填 1
   - `transform_state`：0~100 的百分比

### 需要注意的坑

- **`desire_lifter_pos` 默认值必须是 255**，否则上电瞬间可能误触发升降
- `transform_state` 是 float，在 55 偏移处**非 4 字节对齐**，ARM 上请用 `memcpy` 读写，不要直接 `*(float*)(buf+55)`
- 旧的 `mode` 语义不变，但导航接管底盘时建议约定 `mode=1` 且 `yaw/pitch` 保持当前值，避免云台跳动

---

## 7. 联调验证步骤

1. **单向**：只让 C 板发，上位机看能否稳定解析出 `yaw`、`q`、`lifter_current_pos`
2. **CRC**：故意改坏一个字节，确认两端都丢弃
3. **字段映射**：上位机发 `desire_lifter_pos=1`，C 板回 `lifter_current_pos=1` 且 `transform_state` 从 0 涨到 100
4. **安全值**：发 `desire_lifter_pos=255`，确认升降不动
5. **全零复位**：断开发送后 C 板应在 1 秒内回到 `mode=0` 的静止状态
