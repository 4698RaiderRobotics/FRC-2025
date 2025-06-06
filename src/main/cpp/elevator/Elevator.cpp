
#include <units/math.h>

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "Robot.h"
#include "Constants.h"
#include "util/DataLogger.h"

#include "elevator/Elevator.h"
#include "elevator/ElevatorSim.h"
#include "elevator/ElevatorTalon.h"

using namespace physical::elevator;

Elevator::Elevator()
{
    SetName( "Elevator" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<ElevatorIO> (new ElevatorTalon());
    } else {
        io = std::unique_ptr<ElevatorIO> (new ElevatorSim());
    }

    DataLogger::Log( "Elevator/homingDone", false );

    homer = util::MotorHomer(
        // Start routine
        [this] { io->SetOpenLoop(-0.01); },
        // Stop and Reset Routine
        [this] { 
            io->SetOpenLoop(0.0); 
            io->ResetPosition( -0.125_in ); 
            SetGoal( 0_in ); 
            DataLogger::Log( "Elevator/homingDone", true );
        },
        // Home Condition
        [this] { return units::math::abs( metrics.velocity ) < 0.05_fps; },
        100_ms
    );
}

void Elevator::Periodic() 
{
    io->Update( metrics );
    metrics.Log( "Elevator" );

    // Update the mechanism2d
    // Angle is related to height approximately by height=bar_length*theta
    elevator_lig->SetLength( units::meter_t(metrics.height + 12_in).value() );

    if( frc::DriverStation::IsDisabled() ) {
        SetGoal( metrics.height );
        return;
    }

    homer.Home();
}

void Elevator::SetGoal( units::inch_t goal ) 
{
    if( !homer.isHomingDone() ) {
        return;
    }

    metrics.goal = util::clamp( goal, kElevatorMinHeight, kElevatorMaxHeight );
    io->SetGoal( metrics.goal );
}

void Elevator::Nudge( units::inch_t nudge ) 
{
    SetGoal( metrics.goal + nudge );
}

bool Elevator::AtGoal() 
{
    return units::math::abs( metrics.height - metrics.goal ) < AT_GOAL_TOLERANCE;
}

frc2::CommandPtr Elevator::ChangeHeight( units::inch_t goal ) 
{
    return frc2::cmd::Sequence(
        RunOnce( [this, goal] { SetGoal( goal ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 3_s )
    ).WithName( "Elevator Change Height" );
}

void ElevatorIO::Metrics::Log( const std::string &key ) 
{
    AUTOLOG( key, height );
    AUTOLOG( key, goal );
    AUTOLOG( key, velocity );
    AUTOLOG( key, M1AppliedVolts );
    AUTOLOG( key, M1Current );
    AUTOLOG( key, M2AppliedVolts );
    AUTOLOG( key, M2Current );
}