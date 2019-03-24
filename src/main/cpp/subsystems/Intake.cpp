// Copyright (c) 2018-2019 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

using namespace frc3512;

Intake::Intake() : PublishNode("Intake") {}

void Intake::SetMotors(MotorState motorState) {
    if (motorState == MotorState::kIntake) {
        m_leftMotor.Set(-0.75);
        m_rightMotor.Set(0.75);
    } else if (motorState == MotorState::kOuttake) {
        m_leftMotor.Set(0.75);
        m_rightMotor.Set(-0.75);
    } else {
        m_leftMotor.Set(0.0);
        m_rightMotor.Set(0.0);
    }
}

void Intake::SetClaw(SolenoidState solenoidState) {
    if (solenoidState == SolenoidState::kOpen) {
        m_claw.Set(true);
    } else if (solenoidState == SolenoidState::kClose) {
        m_claw.Set(false);
    }
}

void Intake::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 4 &&
        message.pressed) {
        SetMotors(MotorState::kIntake);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
        message.pressed) {
        SetMotors(MotorState::kOuttake);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 4 &&
        !message.pressed) {
        SetMotors(MotorState::kIdle);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 6 &&
        !message.pressed) {
        SetMotors(MotorState::kIdle);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 3 &&
        message.pressed) {
        SetClaw(SolenoidState::kOpen);
    }
    if (message.topic == "Robot/AppendageStick" && message.button == 5 &&
        message.pressed) {
        SetClaw(SolenoidState::kClose);
    }
}