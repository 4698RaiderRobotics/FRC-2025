#pragma once

#include <frc/geometry/Transform2d.h>
#include <frc2/command/CommandPtr.h>

class Drive;

class DriveCommands {
public:
    static frc2::CommandPtr JoystickDrive( 
        Drive *d, 
        std::function<double()> xSupplier, 
        std::function<double()> ySupplier, 
        std::function<double()> omegaSupplier);

    static frc2::CommandPtr DriveDeltaPose( Drive *d, frc::Transform2d move, bool robotCoords );

private:
    DriveCommands() = default;
};