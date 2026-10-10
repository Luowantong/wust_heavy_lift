namespace mtc {
constexpr double slot_base_square_size = 0.10;             // 底座边长
constexpr double slot_base_height = 0.01;                  // 底座厚度
constexpr double slot_cylinder_height = 0.10;              // 兑换柱高度
constexpr double slot_cylinder_radius = 0.018;             // 兑换柱半径
constexpr double hollow_cylinder_h = 0.15;                 // 能量单元高度
constexpr double hollow_cylinder_r = 0.0325;               // 能量单元半径
constexpr double insert_offset_ = hollow_cylinder_h / 2.0; // 插入深度
constexpr double pre_place_height_ = slot_cylinder_height +
                                     hollow_cylinder_h / 2.0 +
                                     0.01; // 预放置点高度（在插槽上方）
}