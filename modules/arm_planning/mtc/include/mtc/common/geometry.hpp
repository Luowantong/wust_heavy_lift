#pragma once

namespace mtc {
constexpr double slot_cylinder_radius = 0.018;             // 兑换柱半径
constexpr double hollow_cylinder_h = 0.15;                 // 能量单元高度
constexpr double hollow_cylinder_r = 0.0325;               // 能量单元半径
constexpr double insert_offset_ = hollow_cylinder_h / 2.0; // 单元半长（collar 面 → 座位中心）
constexpr double pre_place_height_ = 0.10;
}