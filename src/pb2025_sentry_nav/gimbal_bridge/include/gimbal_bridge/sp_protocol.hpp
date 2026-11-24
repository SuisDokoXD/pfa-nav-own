#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace gimbal_bridge
{

/// CRC16: poly 0x1021 reflected (0x8408), init 0xFFFF, no final xor.
/// Matches tools::get_crc16() in the aim program and CRC16_Calc() on the C board.
inline uint16_t crc16(const uint8_t * data, std::size_t len)
{
  uint16_t crc = 0xFFFF;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 1U) ? static_cast<uint16_t>((crc >> 1) ^ 0x8408U)
                       : static_cast<uint16_t>(crc >> 1);
    }
  }
  return crc;
}

/// Sentinel meaning "leave the lifter where it is".
constexpr uint8_t LIFTER_NO_CHANGE = 0xFFU;
/// Lifter positions, matching the C board enum.
constexpr uint8_t LIFTER_TOP = 0U;
constexpr uint8_t LIFTER_BOTTOM = 1U;
constexpr uint8_t LIFTER_MIDDLE = 2U;

#pragma pack(push, 1)

/// Host -> C board. 51 bytes.
///
/// Layout note: the first 39 bytes are byte-identical to the original
/// navi_minco_bit vision protocol; `crc16` moved from offset 39 to 49 because
/// the tunnel/lifter fields were appended. The C board firmware MUST be rebuilt
/// against this layout, otherwise it will read the CRC from the wrong offset.
struct VisionToGimbal
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode = 0;  // 0 = no control, 1 = control gimbal, 2 = control + fire
  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float yaw_acc = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float pitch_acc = 0.0F;
  float vx = 0.0F;
  float vy = 0.0F;
  float angle_diff = 0.0F;

  // ---- tunnel / lifter extension (new) ----
  float vw = 0.0F;                      // chassis yaw rate, rad/s
  uint8_t desire_lifter_pos = LIFTER_NO_CHANGE;  // 0 top, 1 bottom, 2 middle, 0xFF no change
  uint8_t tunnel_align_active = 0;      // ask the chassis to align its heading
  float tunnel_align_angle_deg = 0.0F;  // target heading for the alignment, degrees

  uint16_t crc16 = 0;
};
static_assert(sizeof(VisionToGimbal) == 51, "VisionToGimbal must be 51 bytes");

/// C board -> host. 61 bytes.
///
/// Layout note: the first 52 bytes match the original protocol; the lifter
/// feedback fields were appended before `crc16`, which moved 52 -> 59.
struct GimbalToVision
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode = 0;
  float q[4] = {1.0F, 0.0F, 0.0F, 0.0F};  // w x y z
  float yaw = 0.0F;
  float yaw_vel = 0.0F;
  float pitch = 0.0F;
  float pitch_vel = 0.0F;
  float bullet_speed = 0.0F;
  uint16_t bullet_count = 0;
  uint16_t projectile_allowance = 0;
  uint16_t current_hp = 0;
  uint16_t current_outpost_hp = 0;
  uint16_t current_base_hp = 0;
  uint8_t game_state = 0;
  uint16_t stage_remain_time = 0;

  // ---- tunnel / lifter feedback (new) ----
  uint8_t lifter_current_pos = LIFTER_NO_CHANGE;  // 0 top, 1 bottom, 2 middle
  uint8_t is_transformable = 1;                   // 0 while moving / jammed
  uint8_t tunnel_yaw_aligned = 0;                 // alignment finished
  float transform_state = 0.0F;                   // 0..100 %

  uint16_t crc16 = 0;
};
static_assert(sizeof(GimbalToVision) == 61, "GimbalToVision must be 61 bytes");

#pragma pack(pop)

inline void finalizeTx(VisionToGimbal & p)
{
  p.crc16 = crc16(reinterpret_cast<const uint8_t *>(&p), sizeof(p) - sizeof(p.crc16));
}

inline bool checkRx(const GimbalToVision & p)
{
  if (p.head[0] != 'S' || p.head[1] != 'P') {
    return false;
  }
  return crc16(reinterpret_cast<const uint8_t *>(&p), sizeof(p) - sizeof(p.crc16)) == p.crc16;
}

}  // namespace gimbal_bridge
