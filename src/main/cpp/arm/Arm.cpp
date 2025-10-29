
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
#include "arm/ArmTalonMinion.h"

using namespace physical::arm;

Arm::Arm()
    : back_rest_angle{ kElbowRestAngle }
{
    SetName( "Arm" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<ArmIO> (new ArmTalonMinion());
    } else {
        io = std::unique_ptr<ArmIO> (new ArmSim());
    }

    DataLogger::Log( "Arm/homingDone", false );

    wristHomer = util::MotorHomer(
        // Start routine
        [this] { io->SetWristOpenLoop(-0.05); },
        // Stop and Reset Routine
        [this] { 
            io->SetWristOpenLoop(0.0); 
            io->ResetWristAngle( 0_deg ); 
            SetWristGoal(ArmIO::WristHorizontal);
            DataLogger::Log( "Arm/homingDone", true );
        },
        // Home Condition
        [this] { return units::math::abs( metrics.wristVelocity ) < 0.1_rpm; },
        200_ms
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
        units::degree_t goal = util::clamp( metrics.elbowPosition, kElbowMinAngle, kElbowHomingRestAngle );
        SetElbowGoal( goal );
        return;
    }

    wristHomer.Home();
}

void Arm::SetElbowGoal( units::degree_t goal ) 
{
    metrics.elbowGoal = util::clamp( goal, kElbowMinAngle, kElbowMaxAngle );
    io->SetElbowGoal( metrics.elbowGoal );
}

void Arm::NudgeElbow( units::degree_t nudge ) {
    SetElbowGoal( metrics.elbowGoal + nudge );
}

ArmIO::WristPosition Arm::GetWristGoal() 
{ 
    if( metrics.wristGoal < 45_deg ) {
        return ArmIO::WristHorizontal;
    } else {
        return ArmIO::WristVertical;
    }
}

void Arm::SetWristGoal( ArmIO::WristPosition pos )
{
    if( !wristHomer.isHomingDone() ) {
        return;
    }

    switch( pos ) {
    case ArmIO::WristHorizontal:
        metrics.wristGoal = kWristHorizontal;
        break;
    case ArmIO::WristVertical:
        metrics.wristGoal = kWristVertical;
        break;
    }

    io->SetWristPosition( pos );
}

bool Arm::AllAtGoal() 
{
    return WristAtGoal() && ElbowAtGoal();
}

bool Arm::ElbowAtGoal() 
{
    return units::math::abs( metrics.elbowPosition - metrics.elbowGoal ) < ELBOW_GOAL_TOLERANCE;
}

bool Arm::WristAtGoal() 
{
    return units::math::abs( metrics.wristPosition - metrics.wristGoal ) < WRIST_GOAL_TOLERANCE;
}

bool Arm::isArmBackward()
{
    return metrics.elbowPosition > 90_deg;
}

frc2::CommandPtr Arm::ChangeElbowAngle( units::degree_t goal ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, goal] { SetElbowGoal( goal ); }),
        frc2::cmd::WaitUntil( [this] { return ElbowAtGoal(); } ).WithTimeout( 1_s )
    ).WithName( fmt::format( "Change Arm Angle to {}", goal ) );
}

frc2::CommandPtr Arm::ChangeWristPosition( ArmIO::WristPosition pos ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, pos] { SetWristGoal( pos ); }),
        frc2::cmd::WaitUntil( [this] { return WristAtGoal(); } ).WithTimeout( 1_s )
    ).WithName( "Change Wrist Position" );
}

frc2::CommandPtr Arm::ChangeElbowAndWrist( units::degree_t elbow_goal, ArmIO::WristPosition pos ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, elbow_goal] { SetElbowGoal( elbow_goal ); }),
        RunOnce( [this, pos] { SetWristGoal( pos ); }),
        frc2::cmd::WaitUntil( [this] { return ElbowAtGoal() && WristAtGoal(); } ).WithTimeout( 1_s )
    ).WithName( "ChangeElbowAndWrist" );
}

units::degree_t Arm::GetElbowRest()
{
    return back_rest_angle;
}

frc2::CommandPtr Arm::SetClimberHoming( bool isClimberHoming )
{
    // Do not make this command have the Arm subsystem as a requirement as that 
    // will break the Autonomous commands by interrupting them with this command.
    // Use frc2::cmd::RunOnce() without requirments.
    return frc2::cmd::RunOnce( [this, isClimberHoming] { 
        if( isClimberHoming ) {
            back_rest_angle = kElbowHomingRestAngle;
        } else {
            back_rest_angle = kElbowRestAngle;
        }
    });
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

