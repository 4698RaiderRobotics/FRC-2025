
#include <numbers>
#include <cmath>
#include <units/angle.h>
#include <units/angular_acceleration.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "util/DataLogger.h"
#include "swerve/SwerveConstants.h"
#include "swerve/Module.h"


Module::Module( ModuleIO* io ) : io{std::move(io)}
{
    m_name = fmt::format( "/Module{}", io->GetIndex() );
}

// Sets each individual SwerveModule to an optimized SwerveModuleState
void Module::Periodic( ) {
    inputs.LogInputs( "Swerve" + m_name );

    // On first cycle reset the motor encoder to the absolute encoder value
    if( !turnRelativeOffset && units::math::fabs(inputs.turnAbsolutePosition) > 1E-6_rad ) {
        turnRelativeOffset = inputs.turnAbsolutePosition - inputs.turnPosition;
        DataLogger::Log( "Swerve" + m_name + "/turnRelativeOffset", turnRelativeOffset );
    }

    // Calculate positions for odometry
    size_t sampleCount = inputs.odometryTimestamps.size();
    odometryPositions.clear();
    for( size_t i=0; i<sampleCount; ++i ) {
        units::meter_t position = inputs.odometryDrivePositions[i] * swerve::physical::kDriveMetersPerWheelRotation;
        units::radian_t angle = inputs.odometryTurnPositions[i] + turnRelativeOffset.value_or(0_rad);
        odometryPositions.push_back( frc::SwerveModulePosition( position, angle ) );
    }
}

// Calculate the angle and speed setpoints
frc::SwerveModuleState Module::RunSetpoint( const frc::SwerveModuleState& state ) {

    // Optimize the state based on the current module turn angle
    frc::SwerveModuleState optimizedState = state;
    optimizedState.Optimize( GetAngle() );

    // Run closed loop control on angle
    io->SetTurnPosition( optimizedState.angle.Radians() );

    // Run closed loop control on speed
    units::radian_t angleError = optimizedState.angle.Radians() - GetAngle();
    optimizedState.speed = optimizedState.speed * std::pow( units::math::cos( angleError ).value(), 3);

    // speedSetpoint = optimizedState.speed;
    units::radians_per_second_t wheelVelocity = optimizedState.speed / swerve::physical::kDriveMetersPerWheelRotation;
    
    io->SetDriveWheelVelocity( wheelVelocity );

    return optimizedState;
}

void Module::RunCharacterization( const units::volt_t volts ) {
        // Keep wheels straight
    io->SetTurnPosition( 0_deg );

        // Use open loop velocity control
    io->SetDriveOpenLoop( volts / 12_V );
}

units::radian_t Module::GetAngle() {
    if( turnRelativeOffset ) {
        return inputs.turnPosition + turnRelativeOffset.value();
    } else {
        return 0_rad;
    }
}

// Returns the current state of the SwerveModule
frc::SwerveModuleState Module::GetState( void ) {
    return { inputs.driveVelocity * swerve::physical::kDriveMetersPerWheelRotation, GetAngle() };
}

frc::SwerveModulePosition Module::GetPosition( void ) {
    return { inputs.drivePosition * swerve::physical::kDriveMetersPerWheelRotation, GetAngle() };
}

void Module::Stop() {
    io->SetTurnOpenLoop( 0.0 );
    io->SetDriveOpenLoop( 0.0 );
}

void Module::UpdateInputs() {
    io->UpdateInputs( inputs );
}

void ModuleIO::Inputs::LogInputs( std::string key ) {
    AUTOLOG( key, drivePosition )
    AUTOLOG( key, driveVelocity )
    AUTOLOG( key, driveAppliedVolts )
    AUTOLOG( key, driveCurrent )

    AUTOLOG( key, turnAbsolutePosition )
    AUTOLOG( key, turnPosition )
    AUTOLOG( key, turnVelocity )
    AUTOLOG( key, turnAppliedVolts )
    AUTOLOG( key, turnCurrent )

    AUTOLOG( key, odometryTimestamps )
    AUTOLOG( key, odometryDrivePositions )
    AUTOLOG( key, odometryTurnPositions )
}
