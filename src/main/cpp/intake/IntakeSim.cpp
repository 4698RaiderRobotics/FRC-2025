
#include <frc/Timer.h>

#include "intake/Intake.h"
#include "intake/IntakeSim.h"

IntakeSim::IntakeSim()
{

}

void IntakeSim::Update( Metrics &m )
{
    upperRoller.Update();
    lowerRoller.Update();
    CheckIntakeEjectState();

    m.upperPosition = upperRoller.GetPosition();
    m.upperVelocity = upperRoller.GetVelocity();
    m.upperAppliedVolts = upperRoller.GetVoltage();
    m.upperCurrent = upperRoller.GetCurrent();

    m.lowerPosition = lowerRoller.GetPosition();
    m.lowerVelocity = lowerRoller.GetVelocity();
    m.lowerAppliedVolts = lowerRoller.GetVoltage();
    m.lowerCurrent = lowerRoller.GetCurrent();

    m.centerBeamBroken = centerBeamBlocked;
    m.endBeamBroken = endBeamBlocked;
    m.pipeSwitchTripped = pipeSwitchTripped;
}

void IntakeSim::SpinMotors( const SpinSpeed &s )
{
    upperRoller.SetOpenLoop( s.upperSpeed );
    lowerRoller.SetOpenLoop( s.lowerSpeed );

    StartIntakeEjectState( s );
}

bool IntakeSim::isEqual( const SpinSpeed &s1, const SpinSpeed &s2 )
{
    return fabs( s1.upperSpeed - s2.upperSpeed ) < 0.0001 && fabs( s1.lowerSpeed - s2.lowerSpeed ) < 0.0001;
}

void IntakeSim::StartIntakeEjectState( const SpinSpeed &s )
{
    // Do some trickery to simulate the coral beam breaks being tripped.
    if( isEqual( s, spin_in ) ) {
        // We are spinning a coral in...
        spinning_in = true;
        spinning_out = false;
        shifting_up = false;
        shifting_down = false;
        start_time = frc::Timer::GetFPGATimestamp();
    } else if( isEqual( s, spin_out ) ) {
        // We are spiting a coral out...
        spinning_in = false;
        spinning_out = true;
        shifting_up = false;
        shifting_down = false;
        start_time = frc::Timer::GetFPGATimestamp();
    } else if( isEqual( s, shift_up ) ) {
        // We are shifting a coral up...
        spinning_in = false;
        spinning_out = false;
        shifting_up = true;
        shifting_down = false;
        start_time = frc::Timer::GetFPGATimestamp();
    } else if( isEqual( s, shift_down ) ) {
        // We are shifting a coral down...
        spinning_in = false;
        spinning_out = false;
        shifting_up = false;
        shifting_down = true;
        start_time = frc::Timer::GetFPGATimestamp();
    } else {
        // We have stopped most likely
        spinning_in = false;
        spinning_out = false;
        shifting_up = false;
        shifting_down = false;
    }
}

void IntakeSim::CheckIntakeEjectState()
{
    if( spinning_in && frc::Timer::GetFPGATimestamp() - start_time > 1.5_s ) {
        // We have been spinning the intake in for 1.5 seconds... Trip the beam break
        centerBeamBlocked = true;
    } else if( spinning_out && frc::Timer::GetFPGATimestamp() - start_time > 0.5_s ) {
        // We have been spinning the intake out for 0.5 seconds... unTrip the beam break
        centerBeamBlocked = false;
        endBeamBlocked = false;
        pipeSwitchTripped = false;
        polling_pipe = false;
    } else if( shifting_up && frc::Timer::GetFPGATimestamp() - start_time > 0.5_s ) {
        // We have been shifting the coral up for 0.5 seconds... Trip the end beam break
        endBeamBlocked = true;
    } else if( shifting_down && frc::Timer::GetFPGATimestamp() - start_time > 0.2_s ) {
        // We have been shifting the coral up for 0.2 seconds... Trip the end beam break
        endBeamBlocked = false;
        pipeSwitchTripped = false;
        polling_pipe = false;
        if( frc::Timer::GetFPGATimestamp() - start_time > 0.4_s ) {
            centerBeamBlocked = false;
        }
    }

}

void IntakeSim::PollingPipeSwitch()
{
    if( !polling_pipe ) {
        start_time = frc::Timer::GetFPGATimestamp();
        polling_pipe = true;
    }

    if( frc::Timer::GetFPGATimestamp() - start_time > 0.75_s ) {
        pipeSwitchTripped = true;
    }
}
