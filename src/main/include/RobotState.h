
#pragma once

#include <frc/geometry/Pose3d.h>

class Arm;
class Climber;
class Elevator;

class RobotState {
public:
    RobotState( Arm *, Elevator *, Climber * );

    void UpdateLoop();

private:
    Arm *m_arm;
    Elevator *m_elev;
    Climber *m_climb;
};
