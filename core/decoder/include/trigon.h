#ifndef ROBSENSE_LIDAR_TRIGON_H
#define ROBSENSE_LIDAR_TRIGON_H

#include <common/rs_common.h>

#include <cmath>

namespace robosense
{
namespace lidar
{
//#define DBG

class Trigon
{
public:
  constexpr static int32_t kAngleMin = -9000;
  constexpr static int32_t kAngleMax = 45000;

  Trigon()
  {
    int32_t range = kAngleMax - kAngleMin;

    o_sins_ = (float*)malloc(range * sizeof(float));
    o_coss_ = (float*)malloc(range * sizeof(float));

    for (int32_t i = kAngleMin, j = 0; i < kAngleMax; i++, j++)
    {
      double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.01);

      o_sins_[j] = (float)std::sin(rad);
      o_coss_[j] = (float)std::cos(rad);
    }

    sins_ = o_sins_ - kAngleMin;
    coss_ = o_coss_ - kAngleMin;
  }

  ~Trigon()
  {
    free(o_coss_);
    free(o_sins_);
  }

  float sin(int32_t angle)
  {
    if (angle < kAngleMin || angle >= kAngleMax)
    {
      angle = 0;
    }

    return sins_[angle];
  }

  float cos(int32_t angle)
  {
    if (angle < kAngleMin || angle >= kAngleMax)
    {
      angle = 0;
    }

    return coss_[angle];
  }

  void print()
  {
    for (int32_t i = -10; i < 10; i++)
    {
      std::cout << sins_[i] << "\t" << coss_[i] << std::endl;
    }
  }

private:
  float* o_sins_;
  float* o_coss_;
  float* sins_;
  float* coss_;
};

}  // namespace lidar
}  // namespace robosense

#endif  // ROBSENSE_LIDAR_TRIGON_H
