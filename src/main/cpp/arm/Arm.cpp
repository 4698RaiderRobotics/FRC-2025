
#include <units/math.h>

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "util/DataLogger.h"
#include "util/Utility.h"

#include "arm/Arm.h"
#include "arm/ArmSim.h"
#include "arm/ArmTalon550.h"

using namespace physical::arm;

Arm::Arm()
{
    SetName( "Arm" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<ArmIO> (new ArmTalon550());
    } else {
        io = std::unique_ptr<ArmIO> (new ArmSim());
    }
}

void Arm::Periodic() 
{
    io->Update( metrics );
    metrics.Log( "Arm" );

    if( frc::DriverStation::IsDisabled() ) {
        metrics.elbowGoal = metrics.elbowPosition;
        return;
    }

}

void Arm::SetGoal( units::degree_t goal ) 
{
    metrics.elbowGoal = util::clamp( goal, kElbowMinAngle, kElbowMaxAngle );
    io->SetElbowGoal( metrics.elbowGoal );
}

void Arm::Nudge( units::degree_t nudge ) {
    SetGoal( metrics.elbowGoal + nudge );
}

bool Arm::AtGoal() 
{
    return units::math::abs( metrics.elbowPosition - metrics.elbowGoal ) < AT_GOAL_TOLERANCE;
}

frc2::CommandPtr Arm::ChangeAngle( units::degree_t goal ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, goal] { SetGoal( goal ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 2_s )
    ).WithName( fmt::format( "Arm Change Angle {}", goal ) );
}

void ArmIO::Metrics::Log( const std::string &key ) 
{
    AUTOLOG( key, elbowPosition );
    AUTOLOG( key, elbowGoal );
    AUTOLOG( key, elbowVelocity );
    AUTOLOG( key, elbowEncPosition );
    AUTOLOG( key, elbowAppliedVolts );
    AUTOLOG( key, elbowCurrent );

    AUTOLOG( key, wristPosition );
    AUTOLOG( key, wristGoal );
    AUTOLOG( key, wristVelocity );
    AUTOLOG( key, wristAppliedVolts );
    AUTOLOG( key, wristCurrent );
}

