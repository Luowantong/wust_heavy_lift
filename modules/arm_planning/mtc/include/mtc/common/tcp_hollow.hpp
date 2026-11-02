#pragma once
#include <cmath>
#include <tf2/LinearMath/Transform.h>
namespace mtc {
inline tf2::Transform makeTcpHollow() {
  tf2::Transform t;
  t.setOrigin(tf2::Vector3(0.0, 0, 0.075));
  tf2::Quaternion q; 
  q.setRPY(0, M_PI, 0);
  t.setRotation(q);
  return t;
}
}  // namespace mtc