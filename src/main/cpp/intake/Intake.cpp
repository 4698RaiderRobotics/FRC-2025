
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

#include "util/DataLogger.h"

#include "intake/Intake.h"
#include "intake/IntakeSim.h"
#include "intake/Intake550.h"

using namespace physical::intake;

const IntakeIO::SpinSpeed IntakeIO::spin_in{ -kIntakeInSpeed, -kIntakeInSpeed };
const IntakeIO::SpinSpeed IntakeIO::spin_out{ kIntakeOutSpeed, kIntakeOutSpeed };
const IntakeIO::SpinSpeed IntakeIO::shift_up{ kIntakeShiftSlowSpeed, -kIntakeShiftFastSpeed };
const IntakeIO::SpinSpeed IntakeIO::shift_down{ -kIntakeShiftFastSpeed, kIntakeShiftSlowSpeed };
const IntakeIO::SpinSpeed IntakeIO::spin_stop{ 0.0, 0.0 };

Intake::Intake() 
{
    SetName( "Intake" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<Intake550> (new Intake550());
    } else {
        io = std::unique_ptr<IntakeSim> (new IntakeSim());
    }

}

void Intake::Periodic()
{
    io->Update( metrics );
    metrics.Log( "Intake" );
}

void Intake::SpinIn()
{
    io->SpinMotors( IntakeIO::spin_in );
}

void Intake::SpinOut()
{
    io->SpinMotors( IntakeIO::spin_out );
}

void Intake::ShiftUp()
{
    io->SpinMotors( IntakeIO::shift_up );
}

void Intake::ShiftDown()
{
    io->SpinMotors( IntakeIO::shift_down );
}

void Intake::Stop()
{
    io->SpinMotors( IntakeIO::spin_stop );
}

bool Intake::isCenterBroken()
{
    return metrics.centerBeamBroken;
}

bool Intake::isEndBroken()
{
    return metrics.endBeamBroken;
}

bool Intake::isPipeTripped()
{
    // This is just for simulation and does nothing
    // on the real robot.
    io->PollingPipeSwitch();
    
    return metrics.pipeSwitchTripped;
}
frc2::CommandPtr Intake::IntakeAlgae()
{
    return frc2::cmd::Sequence( 
        RunOnce( [this] { SpinIn(); }),
        frc2::cmd::WaitUntil( [this] { return units::math::abs(metrics.lowerVelocity) < 1_rpm;} )
            .WithTimeout( 10_s ),
        RunOnce( [this] { Stop(); })
    );
}

frc2::CommandPtr Intake::IntakeCoral()
{
    return frc2::cmd::Sequence( 
        RunOnce( [this] { SpinIn(); }),
        frc2::cmd::WaitUntil( [this] { return isCenterBroken();} ).WithTimeout( 10_s ),
        frc2::cmd::Either( 
            // If Coral is in intake.  Need to shift it up to the end.
            frc2::cmd::Sequence( 
                RunOnce( [this] { ShiftUp(); }),
                frc2::cmd::WaitUntil( [this] { return isEndBroken();} ).WithTimeout( 1_s )
            ), 
            // If No Coral in the intake
            frc2::cmd::None(),
            [this] {return isCenterBroken(); }
        ),
        RunOnce( [this] { Stop(); })
    );
}

frc2::CommandPtr Intake::EjectCoralL1()
{
    return frc2::cmd::Sequence( 
        RunOnce( [this] { SpinOut(); }),
//            .Until( [this] { return !isCenterBroken(); } )
//            .WithTimeout( 1_s ),
        frc2::cmd::Wait( 0.5_s ),
        RunOnce( [this] { Stop(); })
    ).WithName( "Eject Coral L1");
}

frc2::CommandPtr Intake::EjectCoralL2_4( bool waitForPipeSwitch )
{
    return frc2::cmd::Sequence( 
        frc2::cmd::WaitUntil( [this, waitForPipeSwitch] {return isPipeTripped() || !waitForPipeSwitch;} ).WithTimeout( 1.5_s ),
        frc2::cmd::Either( 
            // If Pipe Switch tripped (instead of timed out) then Eject coral.
            frc2::cmd::Sequence( 
                frc2::cmd::RunOnce( [this] { ShiftDown(); }),
                frc2::cmd::Wait( 0.5_s ),
                frc2::cmd::RunOnce( [this] { Stop(); })
            ), 
            // If Pipe Switch not tripped tripped
            frc2::cmd::None(),
            [this, waitForPipeSwitch] {return isPipeTripped() || !waitForPipeSwitch; }
        )
    ).WithName( "Eject Coral L2-4");

}

void IntakeIO::Metrics::Log( const std::string &key ) 
{
    AUTOLOG( key, upperPosition );
    AUTOLOG( key, lowerPosition );
    AUTOLOG( key, upperVelocity );
    AUTOLOG( key, lowerVelocity );
    AUTOLOG( key, upperAppliedVolts );
    AUTOLOG( key, lowerAppliedVolts );
    AUTOLOG( key, upperCurrent );
    AUTOLOG( key, lowerCurrent );
    AUTOLOG( key, centerBeamBroken );
    AUTOLOG( key, endBeamBroken );
    AUTOLOG( key, pipeSwitchTripped );
}

