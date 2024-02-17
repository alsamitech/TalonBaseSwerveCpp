// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/SwerveMod.h"
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>

SwerveMod::SwerveMod(SwerveModuleInfo modinfo, SwerveModuleConfigs configs){
    using namespace ctre::phoenix6::hardware;
    using namespace ctre::phoenix6::configs;
    this->info=modinfo;
    DriveMotor =new TalonFX(info.DriveMotorID);
    DriveMotor->GetConfigurator().Apply(configs.DriveMotorConfiguration);
    DriveMotorFeedForward=frc::SimpleMotorFeedforward(gains.Drive.kS, gains.Drive.kV, gains.Drive.kA);
    //DriveMotor->GetConfigurator().SetPosition(unit<turn_t>(0));
    AngleMotor= new TalonFX(info.AngleMotorID);
    AngleMotor->GetConfigurator().Apply(configs.AngleMotorConfiguration);
    AngleEncoder = new CANcoder(info.AngleEncoderID);
    AngleEncoder->GetConfigurator().Apply(configs.AngleEncoderConfiguration);

};

SwerveMod::SwerveMod(SwerveModuleInfo modinfo, SwerveModuleConfigs configs, SwerveGains gains){
   this->gains=gains;
   SwerveMod(modinfo, configs); 

}
frc::SwerveModulePosition SwerveMod::getPosition(){
    return frc::SwerveModulePosition{
        units::meter_t{DriveMotor->GetPosition().GetValueAsDouble() *info.Wheel.diameter*3.14159},
        frc::Rotation2d(units::radian_t{AngleMotor->GetPosition().GetValueAsDouble()*(3.14159*2)})}; 

}
frc::Rotation2d SwerveMod::getRotation2d(){
    return frc::Rotation2d(units::radian_t{this->AngleEncoder->GetAbsolutePosition().GetValueAsDouble()*(3.14159*2)});
}
double SwerveMod::getRotations(){
    return this->AngleEncoder->GetAbsolutePosition().GetValueAsDouble();
}
void SwerveMod::resetToAbsoloute(){
    AngleMotor->SetPosition(units::angle::turn_t{getRotations()-info.AngleOffset});
    

}
void SwerveMod::setSpeed(frc::SwerveModuleState goalState, bool openLoop){
    if(openLoop){
        // quod?
        DriveMotorOutput->Output=units::dimensionless::scalar_t{goalState.speed/5.12};
        DriveMotor->SetControl(*DriveMotorOutput);
    }else{
        DriveMotorVelocity->Velocity=units::angular_velocity::turns_per_second_t{goalState.speed/(info.Wheel.diameter*3.14159)};
        DriveMotorVelocity->FeedForward=DriveMotorFeedForward.Calculate(goalState.speed);
        DriveMotor->SetControl(*DriveMotorVelocity);
    }
}
void SwerveMod::setGoalState(frc::SwerveModuleState goalState, bool openLoop){
    goalState=frc::SwerveModuleState::Optimize(goalState, frc::Rotation2d{units::radian_t{AngleMotor->GetPosition().GetValueAsDouble()}});
    AngleMotor->SetControl(AngleMotorPosition->WithPosition(units::angle::turn_t{goalState.angle.Radians()/(3.14159*2)}));
    setSpeed(goalState, openLoop);
    // ego thelo meg
}