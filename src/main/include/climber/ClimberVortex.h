#pragma once

#include "ClimberIO.h"

#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/TrapezoidProfile.h>

#include <rev/SparkFlex.h>

class ClimberVortex : public ClimberIO {
public:
    ClimberVortex( );

    void Update( Metrics &m ) override;

    void SetGoal( units::inch_t goal ) override;
    void SetOpenLoop( double percent ) override;
    void ResetHeight( ) override;

private:
    rev::spark::SparkFlex flex;

    frc::PIDController m_PID;
    frc::SimpleMotorFeedforward<units::meters> m_simpleFF;

    frc::TrapezoidProfile<units::meters> m_Profile;
    frc::TrapezoidProfile<units::meters>::State m_Goal;
    frc::TrapezoidProfile<units::meters>::State m_Setpoint;

    bool isOpenLoop;
};
