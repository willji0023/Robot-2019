/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc3512/drive/DifferentialDrive.h"

#include <algorithm>
#include <cmath>

#include <hal/HAL.h>

using namespace frc3512;

DifferentialDrive::Output DifferentialDrive::ArcadeDrive(double xSpeed,
                                                         double zRotation,
                                                         bool squareInputs) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
               HALUsageReporting::kRobotDrive2_DifferentialArcade);
    reported = true;
  }

  xSpeed = Limit(xSpeed);
  xSpeed = ApplyDeadband(xSpeed, m_deadband);

  zRotation = Limit(zRotation);
  zRotation = ApplyDeadband(zRotation, m_deadband);

  // Square the inputs (while preserving the sign) to increase fine control
  // while permitting full power.
  if (squareInputs) {
    xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
    zRotation = std::copysign(zRotation * zRotation, zRotation);
  }

  double leftMotorOutput;
  double rightMotorOutput;

  double maxInput =
      std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

  if (xSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    } else {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    }
  } else {
    // Third quadrant, else fourth quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    } else {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    }
  }

  return Output(
      Limit(leftMotorOutput) * m_maxOutput,
      Limit(rightMotorOutput) * m_maxOutput * m_rightSideInvertMultiplier);
}

DifferentialDrive::Output DifferentialDrive::CurvatureDrive(double xSpeed,
                                                            double zRotation,
                                                            bool isQuickTurn) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
               HALUsageReporting::kRobotDrive2_DifferentialCurvature);
    reported = true;
  }

  xSpeed = Limit(xSpeed);
  xSpeed = ApplyDeadband(xSpeed, m_deadband);

  zRotation = Limit(zRotation);
  zRotation = ApplyDeadband(zRotation, m_deadband);

  double angularPower;
  bool overPower;

  if (isQuickTurn) {
    if (std::abs(xSpeed) < m_quickStopThreshold) {
      m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator +
                               m_quickStopAlpha * Limit(zRotation) * 2;
    }
    overPower = true;
    angularPower = zRotation;
  } else {
    overPower = false;
    angularPower = std::abs(xSpeed) * zRotation - m_quickStopAccumulator;

    if (m_quickStopAccumulator > 1) {
      m_quickStopAccumulator -= 1;
    } else if (m_quickStopAccumulator < -1) {
      m_quickStopAccumulator += 1;
    } else {
      m_quickStopAccumulator = 0.0;
    }
  }

  double leftMotorOutput = xSpeed + angularPower;
  double rightMotorOutput = xSpeed - angularPower;

  // If rotation is overpowered, reduce both outputs to within acceptable range
  if (overPower) {
    if (leftMotorOutput > 1.0) {
      rightMotorOutput -= leftMotorOutput - 1.0;
      leftMotorOutput = 1.0;
    } else if (rightMotorOutput > 1.0) {
      leftMotorOutput -= rightMotorOutput - 1.0;
      rightMotorOutput = 1.0;
    } else if (leftMotorOutput < -1.0) {
      rightMotorOutput -= leftMotorOutput + 1.0;
      leftMotorOutput = -1.0;
    } else if (rightMotorOutput < -1.0) {
      leftMotorOutput -= rightMotorOutput + 1.0;
      rightMotorOutput = -1.0;
    }
  }

  // Normalize the wheel speeds
  double maxMagnitude =
      std::max(std::abs(leftMotorOutput), std::abs(rightMotorOutput));
  if (maxMagnitude > 1.0) {
    leftMotorOutput /= maxMagnitude;
    rightMotorOutput /= maxMagnitude;
  }

  return Output(leftMotorOutput * m_maxOutput,
                rightMotorOutput * m_maxOutput * m_rightSideInvertMultiplier);
}

DifferentialDrive::Output DifferentialDrive::TankDrive(double leftSpeed,
                                                       double rightSpeed,
                                                       bool squareInputs) {
  static bool reported = false;
  if (!reported) {
    HAL_Report(HALUsageReporting::kResourceType_RobotDrive, 2,
               HALUsageReporting::kRobotDrive2_DifferentialTank);
    reported = true;
  }

  leftSpeed = Limit(leftSpeed);
  leftSpeed = ApplyDeadband(leftSpeed, m_deadband);

  rightSpeed = Limit(rightSpeed);
  rightSpeed = ApplyDeadband(rightSpeed, m_deadband);

  // Square the inputs (while preserving the sign) to increase fine control
  // while permitting full power.
  if (squareInputs) {
    leftSpeed = std::copysign(leftSpeed * leftSpeed, leftSpeed);
    rightSpeed = std::copysign(rightSpeed * rightSpeed, rightSpeed);
  }

  return Output(leftSpeed * m_maxOutput,
                rightSpeed * m_maxOutput * m_rightSideInvertMultiplier);
}

void DifferentialDrive::SetQuickStopThreshold(double threshold) {
  m_quickStopThreshold = threshold;
}

void DifferentialDrive::SetQuickStopAlpha(double alpha) {
  m_quickStopAlpha = alpha;
}

bool DifferentialDrive::IsRightSideInverted() const {
  return m_rightSideInvertMultiplier == -1.0;
}

void DifferentialDrive::SetRightSideInverted(bool rightSideInverted) {
  m_rightSideInvertMultiplier = rightSideInverted ? -1.0 : 1.0;
}
