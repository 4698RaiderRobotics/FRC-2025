#pragma once

#include <frc/geometry/Pose3d.h>

#include "util/LoggedCommand.h"


class ExampleSubsystem;
class Drive;
class Intake;

/**
 * Visualize the coral in the log file.
 */
class CoralViz : public frc2::CommandHelper<LoggedCommand, CoralViz>  {
public:
  
    explicit CoralViz( std::function<frc::Pose2d()> poseFunc, std::function<bool()> hasCoralFunc );

    void Init() override;
    void Execute() override;
    bool IsFinished() override;

private:
    std::function<frc::Pose2d()> poseFunc;
    std::function<bool()> hasCoralFunc;

    size_t coral_count{0};
    std::vector<frc::Pose3d> coralPoses;
};
