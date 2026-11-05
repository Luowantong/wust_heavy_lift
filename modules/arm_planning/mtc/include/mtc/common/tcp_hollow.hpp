#pragma once
#include <cmath>
#include <tf2/LinearMath/Transform.h>
namespace mtc {
inline tf2::Transform makeTcpHollow(double roll = 0.0,
                                    double pitch = -M_PI / 2,
                                    double yaw = 0.0,
                                    double x = 0.0,
                                    double y = 0.0,
                                    double z = 0.0) {
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(x, y, z));
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  t.setRotation(q);
  return t;
}
}  // namespace mtc