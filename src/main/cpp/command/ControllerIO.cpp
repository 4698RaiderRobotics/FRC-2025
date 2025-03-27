
#include <frc/DriverStation.h>

#include <frc2/command/Commands.h>

#include "command/ControllerIO.h"
#include "command/DriveCommands.h"

#include "Controls.h"

ControllerIO* ControllerIO::singleton = nullptr;

ControllerIO::ControllerIO( Drive *d, frc2::CommandXboxController *drive, frc2::CommandXboxController *oper )
    : drive{d}, driverCtrlr{drive}, operatorCtrlr{oper}
{
}

void ControllerIO::SetupControllerIO( Drive *d, frc2::CommandXboxController *drive, frc2::CommandXboxController *oper )
{
    if( singleton == nullptr ) {
        singleton = new ControllerIO( d, drive, oper );
    }
}

frc2::CommandPtr ControllerIO::JoystickDrive( )
{
    return DriveCommands::JoystickDrive( 
        singleton->drive,
        [] { return -singleton->driverCtrlr->GetHID().GetRawAxis( ctrl::drive_X_axis ); },
        [] { return -singleton->driverCtrlr->GetHID().GetRawAxis( ctrl::drive_Y_axis ); },
        [] { return -singleton->driverCtrlr->GetHID().GetRawAxis( ctrl::drive_theta_axis ); }
    );
}

frc2::CommandPtr ControllerIO::CoralRumble( )
{
    return frc2::cmd::Either(
        frc2::cmd::Sequence(
            frc2::cmd::RunOnce( [] { singleton->operatorCtrlr->SetRumble( frc::GenericHID::kBothRumble, 0.9 ); } ),
            frc2::cmd::RunOnce( [] { singleton->driverCtrlr->SetRumble( frc::GenericHID::kBothRumble, 0.9 ); } ),
            frc2::cmd::Wait( 0.5_s ),
            frc2::cmd::RunOnce( [] { singleton->operatorCtrlr->SetRumble( frc::GenericHID::kBothRumble, 0 ); } ),
            frc2::cmd::RunOnce( [] { singleton->driverCtrlr->SetRumble( frc::GenericHID::kBothRumble, 0 ); } )
        ),
        frc2::cmd::None(),
        [] { return frc::DriverStation::IsTeleopEnabled(); }
    );
}

frc2::CommandPtr ControllerIO::NoLevelSelectedRumble( )
{
    return frc2::cmd::Sequence(
        frc2::cmd::RunOnce( [] { singleton->operatorCtrlr->SetRumble( frc::GenericHID::kRightRumble, 0.5 ); } ),
        frc2::cmd::Wait( 0.5_s ),
        frc2::cmd::RunOnce( [] { singleton->operatorCtrlr->SetRumble( frc::GenericHID::kBothRumble, 0 ); } )
    );
}
