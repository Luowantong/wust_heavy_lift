#pragma once

namespace mtc {
constexpr double slot_cylinder_radius = 0.018;             // 兑换柱半径
constexpr double hollow_cylinder_h = 0.15;                 // 能量单元高度
constexpr double hollow_cylinder_r = 0.0325;               // 能量单元半径
constexpr double insert_offset_ = hollow_cylinder_h / 2.0; // 单元半长（collar 面 → 座位中心）
// 兑换柱法兰面在 E 系的 x 坐标（见 exchange_station.xml 的 E_coll_collar）。
// 座位中心 = slot_collar_x + insert_offset_ = -0.098 + 0.075 = -0.023。
constexpr double slot_collar_x = -0.098;
constexpr double pre_place_height_ = 0.18;
}