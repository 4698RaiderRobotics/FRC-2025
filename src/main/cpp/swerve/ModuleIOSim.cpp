#include <cstdlib>
#include <ctime>
#include <numbers>

#include <units/moment_of_inertia.h>
#include <frc/Timer.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "swerve/SwerveConstants.h"
#include "swerve/ModuleIOSim.h"

ModuleIOSim::ModuleIOSim( const ModuleConfigs& configs ) :
    index{ configs.index },
    driveSim{ frc::LinearSystemId::DCMotorSystem( frc::DCMotor::NEO(), 0.025_kg_sq_m, swerve::physical::kDriveGearRatio ), frc::DCMotor::NEO()},
    turnSim{ frc::LinearSystemId::DCMotorSystem(frc::DCMotor::NEO(), 0.004_kg_sq_m, swerve::physical::kTurnGearRatio), frc::DCMotor::NEO()},
    m_turnPIDController{ simTurn_PID.kP, simTurn_PID.kI, simTurn_PID.kD },
    m_drivePIDController{ simDrive_PID.kP, simDrive_PID.kI, simDrive_PID.kD }
{
    m_driveFF = new frc::SimpleMotorFeedforward<units::radian>{ 
        units::volt_t{simDrive_PID.kS},
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::kv_unit>{simDrive_PID.kV}, 
        units::unit_t<frc::SimpleMotorFeedforward<units::radian>::ka_unit>{simDrive_PID.kA}
    };
    m_turnPIDController.EnableContinuousInput( -std::numbers::pi , std::numbers::pi );    
    
    std::srand(std::time(nullptr));
    double random_number = std::rand() / (1.0 * RAND_MAX );  // [0 - 1.0] range
    turnAbsoluteInitPosition = units::radian_t( random_number * 2.0 * std::numbers::pi );

    position_Setpt = 0_rad;
    velocity_Setpt = 0_rad_per_s;
}

void ModuleIOSim::UpdateInputs(Inputs& inputs) 
{
    driveSim.SetInputVoltage( m_driveFF->Calculate( velocity_Setpt ) 
        + m_drivePIDController.Calculate( driveSim.GetAngularVelocity().value(), velocity_Setpt.value() ) * 12_V );

    units::radian_t currentAngle = turnSim.GetAngularPosition() + turnAbsoluteInitPosition;
    turnSim.SetInputVoltage( m_turnPIDController.Calculate( currentAngle.value(), position_Setpt.value() ) * 12_V );

    driveSim.Update( 20_ms );
    turnSim.Update( 20_ms );

    inputs.drivePosition = driveSim.GetAngularPosition();
    inputs.driveVelocity = driveSim.GetAngularVelocity();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrent = driveSim.GetCurrentDraw();

    inputs.turnAbsolutePosition = turnSim.GetAngularPosition() + turnAbsoluteInitPosition;
    inputs.turnPosition = turnSim.GetAngularPosition();
    inputs.turnVelocity = turnSim.GetAngularVelocity();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrent = turnSim.GetCurrentDraw();

    inputs.odometryTimestamps.clear();
    inputs.odometryTimestamps.push_back( frc::Timer::GetFPGATimestamp() );

    inputs.odometryDrivePositions.clear();
    inputs.odometryDrivePositions.push_back(inputs.drivePosition);

    inputs.odometryTurnPositions.clear();
    inputs.odometryTurnPositions.push_back(inputs.turnPosition);
}

void ModuleIOSim::SetDriveOpenLoop( double percent ) 
{
    driveAppliedVolts = percent * 12_V;
    if( driveAppliedVolts > 12_V ) driveAppliedVolts = 12_V;
    if( driveAppliedVolts < -12_V ) driveAppliedVolts = -12_V;

    driveSim.SetInputVoltage( driveAppliedVolts );
}

void ModuleIOSim::SetTurnOpenLoop( double percent ) 
{
    turnAppliedVolts = percent * 12_V;
    if( turnAppliedVolts > 12_V ) turnAppliedVolts = 12_V;
    if( turnAppliedVolts < -12_V ) turnAppliedVolts = -12_V;

    turnSim.SetInputVoltage( turnAppliedVolts );
}

void ModuleIOSim::SetDriveWheelVelocity( units::radians_per_second_t velocity )
{
    velocity_Setpt = velocity;
    // driveSim.SetInputVoltage( m_driveFF->Calculate( velocity ) 
    //     + m_drivePIDController.Calculate( driveSim.GetAngularVelocity().value(), velocity.value() ) * 12_V );
}

void ModuleIOSim::SetTurnPosition( units::radian_t position )
{
    position_Setpt = position;
    // units::radian_t currentAngle = turnSim.GetAngularPosition() + turnAbsoluteInitPosition;
    // turnSim.SetInputVoltage( m_turnPIDController.Calculate( currentAngle.value(), position.value() ) * 12_V );
}
