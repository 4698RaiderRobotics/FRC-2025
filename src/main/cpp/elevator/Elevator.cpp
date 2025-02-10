
#include <units/math.h>

#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "Constants.h"
#include "util/DataLogger.h"

#include "elevator/Elevator.h"
#include "elevator/ElevatorSim.h"
#include "elevator/ElevatorTalon.h"

Elevator::Elevator()
{
    SetName( "Elevator" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<ElevatorIO> (new ElevatorTalon());
    } else {
        io = std::unique_ptr<ElevatorIO> (new ElevatorSim());
    }
}

void Elevator::Periodic() {
    io->Update( metrics );
    metrics.Log( "Elevator" );
}

void Elevator::SetGoal( units::inch_t goal ) {
    if( goal < 0_in ) {
         metrics.goal = 0_in;
    } else if( goal > physical::kElevatorMaxHeight ) {
         metrics.goal = physical::kElevatorMaxHeight;
    } else {
        metrics.goal = goal;
    }
    io->SetGoal( metrics.goal );
}

void Elevator::Nudge( units::inch_t nudge ) {
    SetGoal( metrics.goal + nudge );
}

bool Elevator::AtGoal() {
    return units::math::abs( metrics.height - metrics.goal ) < AT_GOAL_TOLERANCE;
}

frc2::CommandPtr Elevator::ChangeHeight( units::inch_t goal ) {
    return frc2::cmd::Sequence(
        RunOnce( [this, goal] { SetGoal( goal ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 2_s )
    ).WithName( "ChangeHeight" );
}

void ElevatorIO::Metrics::Log( const std::string &key ) {
    AUTOLOG( key, height );
    AUTOLOG( key, goal );
    AUTOLOG( key, velocity );
    AUTOLOG( key, appliedVolts );
    AUTOLOG( key, current );
}