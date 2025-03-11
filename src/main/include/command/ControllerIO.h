#pragma once

#include <functional>

#include <frc2/command/button/CommandXBoxController.h>

#include <frc2/command/CommandPtr.h>

class Drive;

class ControllerIO {
public:
    static void SetupControllerIO( Drive *d, frc2::CommandXboxController *drive, frc2::CommandXboxController *oper );
    // static frc2::CommandPtr JoystickDrive( 
    //     Drive *d, 
    //     std::function<double()> xSupplier, 
    //     std::function<double()> ySupplier, 
    //     std::function<double()> omegaSupplier);

    static frc2::CommandPtr JoystickDrive( );
    static frc2::CommandPtr CoralRumble( );
    static frc2::CommandPtr NoLevelSelectedRumble( );

    // static frc2::CommandPtr DriveToPosePP_NOTGOOD( Drive *d, std::function<frc::Pose2d()> poseFunc );
    // static frc2::CommandPtr DriveOpenLoop( Drive *d, frc::ChassisSpeeds speed, bool robotRelative );
    // static frc2::CommandPtr DriveDeltaPose( Drive *d, frc::Transform2d move, bool robotRelative, double fractionFullSpeed );

private:
    ControllerIO( Drive *d, frc2::CommandXboxController *drive, frc2::CommandXboxController *oper );

    static ControllerIO *singleton;
    Drive *drive;
    frc2::CommandXboxController *driverCtrlr;
    frc2::CommandXboxController *operatorCtrlr;
};