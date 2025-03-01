
#include <units/math.h>

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "Robot.h"
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

    elbowHomer = util::MotorHomer(
        // Start routine
        [this] { io->SetWristOpenLoop(-0.05); },
        // Stop and Reset Routine
        [this] { io->SetWristOpenLoop(0.0); io->ResetWristAngle( 0_deg ); SetWristPosition(ArmIO::WristHorizontal); },
        // Home Condition
        [this] { return units::math::abs( metrics.wristVelocity ) < 0.5_rpm; }
    );

}

void Arm::Periodic() 
{
    io->Update( metrics );
    metrics.Log( "Arm" );

    // Update the mechanism2d
    elbow_lig->SetAngle( metrics.elbowPosition - 96_deg );
    wrist_lig1->SetLength( (2 + 6 * units::math::sin(metrics.wristPosition))/39.0 );
    wrist_lig2->SetLength( (2 + 6 * units::math::sin(metrics.wristPosition))/39.0 );

    if( frc::DriverStation::IsDisabled() ) {
        SetElbowGoal( metrics.elbowPosition );
        return;
    }

    elbowHomer.Home();
}

void Arm::SetElbowGoal( units::degree_t goal ) 
{
    metrics.elbowGoal = util::clamp( goal, kElbowMinAngle, kElbowMaxAngle );
    io->SetElbowGoal( metrics.elbowGoal );
}

void Arm::NudgeElbow( units::degree_t nudge ) {
    SetElbowGoal( metrics.elbowGoal + nudge );
}

void Arm::SetWristPosition( ArmIO::WristPosition pos )
{
    switch( pos ) {
    case ArmIO::WristHorizontal:
        metrics.wristGoal = -5_deg;
        break;
    case ArmIO::WristVertical:
        metrics.wristGoal = 95_deg;
        break;
    }

    io->SetWristPosition( pos );
}

bool Arm::AtGoal() 
{
    return units::math::abs( metrics.elbowPosition - metrics.elbowGoal ) < AT_GOAL_TOLERANCE &&
           units::math::abs( metrics.wristPosition - metrics.wristGoal ) < AT_GOAL_TOLERANCE;
}

frc2::CommandPtr Arm::ChangeElbowAngle( units::degree_t goal ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, goal] { SetElbowGoal( goal ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 2_s )
    ).WithName( fmt::format( "Change Arm Angle to {}", goal ) );
}

frc2::CommandPtr Arm::ChangeWristPosition( ArmIO::WristPosition pos ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, pos] { SetWristPosition( pos ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 2_s )
    ).WithName( "Change Wrist Position" );
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

