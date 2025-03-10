#pragma once

#include <functional>

#include <frc/geometry/Transform2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <frc2/command/CommandPtr.h>

class Drive;

class DriveCommands {
public:
    static frc2::CommandPtr JoystickDrive( 
        Drive *d, 
        std::function<double()> xSupplier, 
        std::function<double()> ySupplier, 
        std::function<double()> omegaSupplier);

    static frc2::CommandPtr DriveToPosePP( Drive *d, std::function<frc::Pose2d()> poseFunc );
    static frc2::CommandPtr DriveOpenLoop( Drive *d, frc::ChassisSpeeds speed, bool robotRelative );
    static frc2::CommandPtr DriveDeltaPose( Drive *d, frc::Transform2d move, bool robotRelative );

private:
    DriveCommands() = default;
};