#pragma once

#include <string>
#include <units/velocity.h>

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

//#include "AbsoluteEncoder.h"

// Class for each swerve module on the robot
class SwerveModule {
  public:
    SwerveModule( const int turnMotorChannel, const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset );

    void SetDesiredState( const frc::SwerveModuleState& state );

    frc::SwerveModuleState GetState( void );
    frc::SwerveModulePosition GetPosition ( void );

  private:
    std::string m_name;

    ctre::phoenix6::hardware::TalonFX m_driveMotor;
    ctre::phoenix6::controls::VelocityVoltage m_driveVelocity{0_tps};

    ctre::phoenix6::hardware::TalonFX m_turnMotor;
    ctre::phoenix6::controls::PositionDutyCycle m_turnPosition{0_tr};

    ctre::phoenix6::hardware::CANcoder m_turnAbsEncoder;

    // Use a software PID controller and feedforward for the turn motor.
    frc::PIDController m_turnPIDController{ 0, 0, 0 };
    //frc::SimpleMotorFeedforward<units::degrees> m_turnFF;

    frc::SwerveModuleState state;
    units::degrees_per_second_t speed;
    units::degrees_per_second_t opSpeed;
    units::degree_t dTheta;

    double pidOutput;

    friend class SwerveDriveSubsystem;
};