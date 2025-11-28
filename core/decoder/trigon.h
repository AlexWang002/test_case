/*******************************************************************************
 * \addtogroup include
 * \{
 * \headerfile trigon.h "trigon.h"
 * \brief Defines trigonometric functions
 * \version 0.2
 * \date 2025-08-07
 *
 * \copyright (c) 2014 - 2025 RoboSense, Co., Ltd.  All rights reserved.
 *
 * \details
 * #### Modification History :
 * | ver |    date    |  description |
 * |-----|------------|--------------|
 * | 0.1 | 2025-08-07 | Init version |
 * | 0.2 | 2025-08-07 | Add comments |
 *
 ******************************************************************************/
#ifndef I_ROBOSENSE_LIDAR_TRIGON_H
#define I_ROBOSENSE_LIDAR_TRIGON_H

/******************************************************************************/
/*                     Include dependant library headers                      */
/******************************************************************************/
#include <cmath>

/******************************************************************************/
/*                      Include headers of the component                      */
/******************************************************************************/
#include "common/rs_common.h"

/******************************************************************************/
/*                          Definition of namespace                           */
/******************************************************************************/
namespace robosense::lidar {

/******************************************************************************/
/*                   Definition of classes or templates                       */
/******************************************************************************/
/**
 * \class Trigon trigon.h "trigon.h"
 * \brief A fast lookup table class for trigonometric functions.
 */
class Trigon final {
  public:
    static constexpr int32_t kAngleMin { -9000 }; ///< Minimum angle in degrees, -90°
    // FIXME Maximum 270 angle is enough for EMX LiDAR
    static constexpr int32_t kAngleMax { 45000 }; ///< Maximum angle in degrees, 450°
    static constexpr int32_t kRange = kAngleMax - kAngleMin;

    Trigon() {
        o_sins_ = (float*)malloc(kRange * sizeof(float));
        o_coss_ = (float*)malloc(kRange * sizeof(float));

        // Precompute sin and cos values for the range of angles
        for (int32_t i = kAngleMin, j = 0; i < kAngleMax; i++, j++) {
            double rad = DEGREE_TO_RADIAN(static_cast<double>(i) * 0.01);

            o_sins_[j] = (float)std::sin(rad);
            o_coss_[j] = (float)std::cos(rad);
        }

        // NOTE Awesome idea!!!
        // Using pointer offset, we can use sins_[angle] to get sin(angle)
        // without causing array out of bounds due to negative indices.
        sins_ = o_sins_ - kAngleMin;
        coss_ = o_coss_ - kAngleMin;
    }

    ~Trigon() {
        free(o_coss_);
        free(o_sins_);
    }

    /**
     * \brief Get the sin value of the angle.
     * \param[in] angle The angle in degrees.
     *   -# Range    : degree -90 to +450
     *   -# Accuracy : 0.01 degree
     * \return The sin value of the angle.
     */
    float sin(int32_t angle) {
        if (angle < kAngleMin || angle >= kAngleMax) {
            angle = 0;
        }

        return sins_[angle];
    }

    /**
     * \brief Get the cos value of the angle.
     * \param[in] angle The angle in degrees.
     *   -# Range    : degree -90 to +450
     *   -# Accuracy : 0.01 degree
     * \return The cos value of the angle.
     */
    float cos(int32_t angle) {
        if (angle < kAngleMin || angle >= kAngleMax) {
            angle = 0;
        }

        return coss_[angle];
    }

    /**
     * \brief Print the sin and cos values for a range of angles.
     */
    void print() {
        for (int32_t i = -10; i < 10; i++) {
            std::cout << sins_[i] << "\t" << coss_[i] << std::endl;
        }
    }

  private:
    float* o_sins_;
    float* o_coss_;
    float* sins_;
    float* coss_;
};

} // namespace robosense::lidar

/** \} include */
#endif /* I_ROBOSENSE_LIDAR_TRIGON_H */
