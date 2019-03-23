/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <memory>

#include <wpi/ArrayRef.h>
#include <wpi/raw_ostream.h>

namespace frc3512 {

/**
 * Common base class for drive platforms.
 */
class RobotDriveBase {
 public:
  RobotDriveBase() = default;

  RobotDriveBase(RobotDriveBase&&) = default;
  RobotDriveBase& operator=(RobotDriveBase&&) = default;

  /**
   * Sets the deadband applied to the drive inputs (e.g., joystick values).
   *
   * The default value is 0.02. Inputs smaller than the deadband are set to 0.0
   * while inputs larger than the deadband are scaled from 0.0 to 1.0. See
   * ApplyDeadband().
   *
   * @param deadband The deadband to set.
   */
  void SetDeadband(double deadband);

  /**
   * Configure the scaling factor for using RobotDrive with motor controllers in
   * a mode other than PercentVbus or to limit the maximum output.
   *
   * @param maxOutput Multiplied with the output percentage computed by the
   *                  drive functions.
   */
  void SetMaxOutput(double maxOutput);

 protected:
  /**
   * Limit motor values to the -1.0 to +1.0 range.
   */
  double Limit(double number);

  /**
   * Returns 0.0 if the given value is within the specified range around zero.
   * The remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
   *
   * @param value    value to clip
   * @param deadband range around zero
   */
  double ApplyDeadband(double number, double deadband);

  /**
   * Normalize all wheel speeds if the magnitude of any wheel is greater than
   * 1.0.
   */
  void Normalize(wpi::MutableArrayRef<double> wheelSpeeds);

  double m_deadband = 0.02;
  double m_maxOutput = 1.0;
};

}  // namespace frc3512
