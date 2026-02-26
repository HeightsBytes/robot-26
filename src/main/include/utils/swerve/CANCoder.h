// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <units/angle.h>

#include <numbers>

namespace hb {
/**
 * @brief CAN coder class for swerve modules
 * @warning ONLY FOR SWERVE MODULES
 */
class S_CANCoder : private ctre::phoenix6::hardware::CANcoder {
 public:
  explicit S_CANCoder(int id, double offset);

  /**
   * @brief gets the positon of the encoder from [-pi, pi]
   * and applies the offset
   */
  units::radian_t Get();

 private:
  double m_offset;

  int m_ID;
};
}  // namespace hb