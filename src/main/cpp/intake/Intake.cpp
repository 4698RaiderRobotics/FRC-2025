
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

#include "util/DataLogger.h"

#include "intake/Intake.h"
#include "intake/IntakeSim.h"
#include "intake/Intake550.h"


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
    io->SpinMotors( physical::kIntakeInOutSpeed, physical::kIntakeInOutSpeed );
}

void Intake::SpinOut()
{
    io->SpinMotors( -physical::kIntakeInOutSpeed, -physical::kIntakeInOutSpeed );
}

void Intake::ShiftUp()
{
    io->SpinMotors( physical::kIntakeInOutSpeed, -physical::kIntakeShiftSpeed );
}

void Intake::ShiftDown()
{
    io->SpinMotors( -physical::kIntakeShiftSpeed, physical::kIntakeInOutSpeed );
}

void Intake::Stop()
{
    io->SpinMotors( 0.0, 0.0 );
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
     return metrics.pipeSwitchTripped;
}

frc2::CommandPtr Intake::IntakeCoral()
{
    return frc2::cmd::Sequence( 
        frc2::cmd::RunOnce( [this] { SpinIn(); }),
        frc2::cmd::WaitUntil( [this] { return isCenterBroken();} ).WithTimeout( 10_s ),
        frc2::cmd::Either( 
            // If Coral is in intake.  Need to shift it up to the end.
            frc2::cmd::Sequence( 
                frc2::cmd::RunOnce( [this] { ShiftUp(); }),
                frc2::cmd::WaitUntil( [this] { return isEndBroken();} ).WithTimeout( 1_s )
            ), 
            // If No Coral in the intake
            frc2::cmd::None(),
            [this] {return isCenterBroken(); }
        )
    );
}

frc2::CommandPtr Intake::EjectCoral()
{
    return frc2::cmd::Sequence( 
        frc2::cmd::RunOnce( [this] { ShiftDown(); }),
        frc2::cmd::Wait( 0.5_s ),
        frc2::cmd::RunOnce( [this] { SpinOut(); }),
        frc2::cmd::Wait( 0.5_s )
    );

}

void IntakeIO::Metrics::Log( const std::string &key ) {
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

