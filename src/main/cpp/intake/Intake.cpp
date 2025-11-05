
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "Constants.h"

#include "util/DataLogger.h"

#include "intake/Intake.h"
#include "intake/IntakeSim.h"
#include "intake/IntakeMinion.h"

using namespace physical::intake;

const IntakeIO::SpinSpeed IntakeIO::hold_in{ -0.1, -0.05 };
const IntakeIO::SpinSpeed IntakeIO::spin_in{ -kIntakeInSpeed, -kIntakeInSpeed };
const IntakeIO::SpinSpeed IntakeIO::spin_out{ kIntakeOutSpeed, kIntakeOutSpeed };
const IntakeIO::SpinSpeed IntakeIO::spin_out_fast{ kIntakeShiftFastSpeed, kIntakeShiftFastSpeed };
const IntakeIO::SpinSpeed IntakeIO::shift_up{ -kIntakeShiftFastSpeed, kIntakeShiftSlowSpeed };
const IntakeIO::SpinSpeed IntakeIO::shift_up_slow{ -kIntakeShiftFastSpeed * 0.5, kIntakeShiftSlowSpeed * 0.5 };
const IntakeIO::SpinSpeed IntakeIO::stopping_up{ -kIntakeShiftSlowSpeed * 0.5, 0 };
const IntakeIO::SpinSpeed IntakeIO::shift_down{ kIntakeShiftSlowSpeed, -kIntakeShiftFastSpeed };
const IntakeIO::SpinSpeed IntakeIO::spin_stop{ 0.0, 0.0 };

Intake::Intake() 
{
    SetName( "Intake" );

    if( frc::RobotBase::IsReal() ) {
        io = std::unique_ptr<IntakeIO> (new IntakeMinion());
    } else {
        io = std::unique_ptr<IntakeIO> (new IntakeSim());
    }
}

void Intake::Periodic()
{
    io->Update( metrics );
    metrics.Log( "Intake" );

    if( frc::DriverStation::IsDisabled() ) {
        Stop();
        return;
    }
    
    if( metrics.centerBeamBroken && isStopped && !metrics.endBeamBroken ) {
        io->SpinMotors( IntakeIO::hold_in );
    } else if( metrics.centerBeamBroken && isStopped && metrics.endBeamBroken ) {
        io->SpinMotors( IntakeIO::spin_stop );
    }
}
void Intake::enableRetention( bool enable) 
{
    isStopped = enable;
}

void Intake::SpinIn()
{
    isStopped = false;
    io->SpinMotors( IntakeIO::spin_in );
}

void Intake::SpinOut()
{
    isStopped = false;
    io->SpinMotors( IntakeIO::spin_out );
}

void Intake::SpinOutFast()
{
    isStopped = false;
    io->SpinMotors( IntakeIO::spin_out_fast );
}

void Intake::ShiftUp()
{
    isStopped = false;
    io->SpinMotors( IntakeIO::shift_up );
}

void Intake::ShiftUpSlow()
{
    isStopped = false;
    io->SpinMotors( IntakeIO::shift_up_slow );
}

void Intake::ShiftDown()
{
    isStopped = false;
    io->SpinMotors( IntakeIO::shift_down );
}

void Intake::Stop()
{
    isStopped = true;
    // if( metrics.centerBeamBroken ) {
    //     io->SpinMotors( IntakeIO::hold_in );
    // } else {
        io->SpinMotors( IntakeIO::spin_stop );
    // }
}

bool Intake::isCenterBroken()
{
    return metrics.centerBeamBroken;
}

bool Intake::isEndBroken()
{
    return metrics.endBeamBroken;
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
        IntakeCoralNoIndex( 20_s ),
        frc2::cmd::Either( 
            // If Coral is in intake.  Need to shift it up to the end.
            IndexCoral(),
            // If No Coral in the intake
        frc2::cmd::None(),
        [this] {return isCenterBroken(); }
        )
    );
}

frc2::CommandPtr Intake::IntakeCoralNoIndex( units::second_t timeout )
{
    return frc2::cmd::Sequence( 
        RunOnce( [this] { SpinIn(); }),
        frc2::cmd::WaitUntil( [this] { return isCenterBroken() || isEndBroken();} ).WithTimeout( timeout )
        // RunOnce( [this] { Stop(); })
    );
}

frc2::CommandPtr Intake::IndexCoral()
{
    return frc2::cmd::Sequence( 
        RunOnce( [this] { ShiftUp(); }),
        frc2::cmd::WaitUntil( [this] { return endBeamBreakDebouce.Calculate( isEndBroken() );} ).WithTimeout( 1.5_s ),
        StopIndex()
    );
}

frc2::CommandPtr Intake::StopIndex()
{
    return frc2::cmd::Sequence( 
            // Slowly ramp down the indexing speed to prevent the coral
            // from bouncing away from the end beam break.
        RunOnce( [this] { ShiftUpSlow(); } ),
        frc2::cmd::Wait( 100_ms ),
        RunOnce( [this] { io->SpinMotors( IntakeIO::stopping_up); } ),
        frc2::cmd::Wait( 100_ms ),
        StopCmd()
    );
}

frc2::CommandPtr Intake::EjectCoralL1()
{
    return // frc2::cmd::Sequence( 
        Run( [this] { SpinOut(); }
//            .Until( [this] { return !isCenterBroken(); } )
//            .WithTimeout( 1_s ),
        // frc2::cmd::Wait( 0.5_s ),
        // RunOnce( [this] { Stop(); })
    ).WithTimeout( 0.25_s ).WithName( "Eject Coral L1");
}

frc2::CommandPtr Intake::EjectCoralL2_4_Fast()
{
    return frc2::cmd::Sequence(
        // Run( [this] { ShiftDown(); }).Until( [this] { return isEndBroken(); }).WithTimeout(0.5_s),
        Run( [this] { ShiftDown(); }).WithTimeout(0.3_s)
        // frc2::cmd::Wait( 0.2_s)
    );
}

frc2::CommandPtr Intake::StopCmd()
{
    return RunOnce( [this] { Stop(); });
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
}

