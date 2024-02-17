// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CanCoder.hpp>
#include <wpi/MathExtras.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/MathUtil.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/BangBangController.h>

struct SwerveModuleInfo{
   int DriveMotorID,
      AngleMotorID,
      AngleEncoderID;
   double AngleOffset;
   struct {
      double diameter;
   }Wheel;
};

struct SwerveGains{
   struct{
   double kP,
         kI,
         kD;
   units::volt_t kS;
   units::unit_t kV;
   units::unit_t kA;
   }Drive;
   struct{
      double kP,
            kI,
            kD;
   }Angle;
};

struct SwerveModuleConfigs{
   ctre::phoenix6::configs::TalonFXConfiguration DriveMotorConfiguration, AngleMotorConfiguration;
   ctre::phoenix6::configs::CANcoderConfiguration AngleEncoderConfiguration;
};

class SwerveMod {
 private:
    int modno;
    ctre::phoenix6::hardware::TalonFX *AngleMotor, *DriveMotor;
    ctre::phoenix6::hardware::CANcoder* AngleEncoder;
    SwerveModuleInfo info;
    SwerveGains gains;
   ctre::phoenix6::controls::DutyCycleOut* DriveMotorOutput;
   ctre::phoenix6::controls::VelocityVoltage* DriveMotorVelocity;

   ctre::phoenix6::controls::PositionDutyCycle* AngleMotorPosition;

   frc::SimpleMotorFeedforward<units::meters> DriveMotorFeedForward;
   
  public:
  SwerveMod(SwerveModuleInfo modinfo, SwerveModuleConfigs configs);
  SwerveMod(SwerveModuleInfo modinfo, SwerveModuleConfigs configs, SwerveGains gains);
  void setGoalState();
  frc::SwerveModulePosition getPosition();
  frc::Rotation2d getRotation2d();
  double getRotations();
  void setSpeed(frc::SwerveModuleState goalState, bool openLoop);
  void setGoalState(frc::SwerveModuleState goalState, bool openLoop);
  void resetToAbsoloute();
};
