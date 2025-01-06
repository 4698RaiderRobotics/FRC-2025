
#include <numbers>
#include <cmath>
#include <units/angle.h>
#include <units/angular_acceleration.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "SwerveConstants.h"
#include "SwerveModule.h"

#include "DataLogger.h"

using namespace swerve;

SwerveModule::SwerveModule( const int turnMotorChannel, 
                           const int driveMotorChannel, const int absoluteEncoderChannel, const double absoluteEncoderOffset ) 
                           : m_driveMotor{driveMotorChannel, ""},
                           m_turnMotor{turnMotorChannel, ""},
                           m_turnAbsEncoder{absoluteEncoderChannel, ""} 
{
    m_driveMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

    ctre::phoenix6::configs::TalonFXConfiguration driveConfigs{};
    driveConfigs.Slot0.kV = swerve::pidf::kDriveV;
    driveConfigs.Slot0.kP = swerve::pidf::kDriveP;
    driveConfigs.Slot0.kI = swerve::pidf::kDriveI;
    driveConfigs.Slot0.kD = swerve::pidf::kDriveD;
    driveConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_driveMotor.GetConfigurator().Apply(driveConfigs, 50_ms);


    m_turnMotor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});
    
    ctre::phoenix6::configs::TalonFXConfiguration turnConfigs{};
    turnConfigs.Slot0.kP = swerve::pidf::kTurnP;
    turnConfigs.Slot0.kI = swerve::pidf::kTurnI;
    turnConfigs.Slot0.kD = swerve::pidf::kTurnD;
    turnConfigs.Feedback.FeedbackRemoteSensorID = absoluteEncoderChannel;
    turnConfigs.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    turnConfigs.MotorOutput.Inverted = true;
    turnConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    turnConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    m_turnMotor.GetConfigurator().Apply(turnConfigs, 50_ms);

    ctre::phoenix6::configs::CANcoderConfiguration absoluteEncoderConfigs{};
    absoluteEncoderConfigs.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
    m_turnAbsEncoder.GetConfigurator().Apply(absoluteEncoderConfigs, 50_ms);

    m_name = fmt::format( "/Swerve Module({}-{})", turnMotorChannel, driveMotorChannel );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void SwerveModule::SetDesiredState( const frc::SwerveModuleState& referenceState ) {
    units::degree_t turn_angle = units::turn_t{m_turnAbsEncoder.GetAbsolutePosition().GetValueAsDouble()};

    state = frc::SwerveModuleState::Optimize( referenceState, turn_angle );

    // The setpoint rpm for the motor
    speed = state.speed / swerve::physical::kDriveMetersPerRotation;

    // Distance between the setpoint angle and the current angle in degrees
    dTheta =  state.angle.Degrees() - turn_angle;

    // Optimizes the rpm based on how far away the wheel is from pointed the correct direction
    // cos^4 of (the error angle) * the rpm
    opSpeed = speed * std::pow(units::math::cos(dTheta).value(), 4); 


    m_driveVelocity.Slot = 0;
    m_driveMotor.SetControl( m_driveVelocity.WithVelocity(speed) );

    m_turnPosition.Slot = 0;
    m_turnMotor.SetControl( m_turnPosition.WithPosition(state.angle.Degrees()) );
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState SwerveModule::GetState( void ) {
    return { m_driveMotor.GetVelocity().GetValue() * swerve::physical::kDriveMetersPerRotation,
             units::radian_t(m_turnAbsEncoder.GetPosition().GetValue()) };
}

frc::SwerveModulePosition SwerveModule::GetPosition( void ) {
    return { m_driveMotor.GetPosition().GetValue() * swerve::physical::kDriveMetersPerRotation, 
             units::radian_t( m_turnAbsEncoder.GetPosition().GetValue() ) };
}
