
#include <units/math.h>

#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "Robot.h"
#include "Constants.h"
#include "util/DataLogger.h"

#include "climber/Climber.h"
#include "climber/ClimberSim.h"
#include "climber/ClimberVortex.h"

using namespace physical::climber;

Climber::Climber()
{
    SetName( "Climber" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<ClimberIO> (new ClimberVortex());
    } else {
        io = std::unique_ptr<ClimberIO> (new ClimberSim());
    }

    climbHomer = util::MotorHomer(
        // Start routine
        [this] { io->SetOpenLoop(-0.05); },
        // Stop and Reset Routine
        [this] { io->SetOpenLoop(0.0); io->ResetHeight(); SetGoal(kClimberRestHeight); },
        // Home Condition
        [this] { return metrics.homeSwitchTripped; },
        100_ms
    );
}

void Climber::Periodic() {
    io->Update( metrics );
    metrics.Log( "Climber" );

    // Update the mechanism2d
    // Angle is related to height approximately by height=bar_length*theta
    climber_lig->SetAngle( 10_deg + ( metrics.height / 11_in )*1_rad );

    if( frc::DriverStation::IsDisabled() ) {
        SetGoal( metrics.height );
        return;
    }

    climbHomer.Home();
}

void Climber::SetGoal( units::inch_t goal ) 
{
    metrics.goal = util::clamp( goal, kClimberMinHeight, kClimberMaxHeight );
    io->SetGoal( metrics.goal );
}

void Climber::Nudge( units::inch_t nudge ) 
{
    metrics.goal += nudge;
    io->SetGoal( metrics.goal );
}

bool Climber::AtGoal() 
{
    return units::math::abs( metrics.height - metrics.goal ) < AT_GOAL_TOLERANCE;
}

frc2::CommandPtr Climber::RaiseClimber( )
{
    return frc2::cmd::Sequence(
        RunOnce( [this] { SetGoal( kClimberRaiseHeight ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 2_s )
    );
}

frc2::CommandPtr Climber::DoClimb( )
{
    return frc2::cmd::Sequence(
        RunOnce( [this] { SetGoal( kClimberClimbHeight ); }),
        frc2::cmd::WaitUntil( [this] { return AtGoal(); } ).WithTimeout( 2_s )
    );
}

frc2::Trigger Climber::isHoming()
{
    return frc2::Trigger( [this] { return climbHomer.isHomingActive(); } );
}


void ClimberIO::Metrics::Log( const std::string &key )
{
    AUTOLOG( key, height );
    AUTOLOG( key, goal );
    AUTOLOG( key, velocity );
    AUTOLOG( key, appliedVolts );
    AUTOLOG( key, current );
    AUTOLOG( key, homeSwitchTripped );
}