#ifndef WUST_ARM_DRIVER__PACKET_HPP_
#define WUST_ARM_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace wust_arm_driver {
// 接收包：电控回传给电脑（7个角度反馈）
struct ReceivePacket {
  uint8_t header = 0x5A;
  float current_joint_positions[7]; // 当前弧度
  uint16_t crc16;
} __attribute__((packed));

// 发送包：电脑发给电控（7个目标角度）
struct SendPacket {
  uint8_t header = 0xA5;
  float target_joint_positions[7];  // 目标弧度
  float target_joint_velocities[7]; // 目标速度（可选）
  uint16_t crc16;
} __attribute__((packed));

inline ReceivePacket fromVector(const std::vector<uint8_t> &data) {
  ReceivePacket packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

inline std::vector<uint8_t> toVector(const SendPacket &data) {
  std::vector<uint8_t> packet(sizeof(SendPacket));
  const uint8_t *p = reinterpret_cast<const uint8_t *>(&data);
  std::copy(p, p + sizeof(SendPacket), packet.begin());
  return packet;
}
} // namespace wust_arm_driver
#endif