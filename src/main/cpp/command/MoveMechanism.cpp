
#include <numbers>

#include "swerve/Drive.h"

#include "command/MoveMechanism.h"
#include "elevator/Elevator.h"
#include "util/DataLogger.h"

#include "Constants.h"

using namespace physical;

MoveMechanism::MoveMechanism( Arm *arm, Elevator *elevator, units::inch_t height, units::degree_t elbow, ArmIO::WristPosition pos ) 
    : m_arm{arm}, m_elevator{elevator}, height_goal{height}, elbow_goal{elbow}, wrist_goal{pos}
{
    SetName( "MoveMechanism" );

    AddRequirements({m_arm, m_elevator});
}

void MoveMechanism::Init() 
{
    final_move = false;
    delay_for_wrist = false;
    delay_for_elbow = false;

    if( units::math::abs( elbow_goal - arm::kElbowRestAngle ) < 0.1_deg ) {
        elbow_goal = m_arm->GetElbowRest();
    }

    if( m_arm->isArmBackward() ) {
        // Arm is back
        move_arm_backward = false;
        if( elbow_goal > 90_deg ) {
            // We want to go back with the arm
            move_arm_forward = false;
            move_elevator_down = false;
            if( elbow_goal > arm::kElbowRestAngle ) {
                delay_for_elbow = true;
            }
        } else {
            // We want to go forward with the arm
            move_arm_forward = true;
            if( m_elevator->GetHeight() > 1_in ) {
                move_elevator_down = true;
            } else {
                move_elevator_down = false;
            }
        }
    } else {
        // Arm is forward
        move_arm_forward = false;
        if( elbow_goal < 90_deg ) {
            // We want to go forward with the arm
            move_arm_backward = false;
            move_elevator_down = false;
        } else {
            // We want to go back with the arm
            move_arm_backward = true;
            if( m_elevator->GetHeight() > 1_in ) {
                move_elevator_down = true;
            } else {
                move_elevator_down = false;
            }
                // If the elevator is lowish and wrist is vertical and the arm needs to move back
                // then we need to delay a little to let the wrist move horizontal before moving the
                // arm back
            if( m_elevator->GetHeight() < 20_in && 
                m_arm->GetWristGoal() == ArmIO::WristVertical &&
                wrist_goal == ArmIO::WristHorizontal ) {
                delay_for_wrist = true;
            }
        }
    }
}

// Called repeatedly when this Command is scheduled to run
void MoveMechanism::Execute() 
{
    if( move_elevator_down ) {
        if( move_arm_forward ) {
            // Arm is back and needs to come down and forward
            m_arm->SetElbowGoal( arm::kElbowBackwardRaiseAngle );
        } else {
            // Arm is forward and needs to come down and back
            m_arm->SetElbowGoal( arm::kElbowForwardRaiseAngle );
        }
        if( wrist_goal == ArmIO::WristHorizontal ) {
            // if( delay_for_wrist ) {
            //     wrist_start_time = frc::Timer::GetFPGATimestamp();
            // }
            m_arm->SetWristGoal( wrist_goal );
        }
        m_elevator->SetGoal( elevator::kElevatorMinHeight );
        if( m_elevator->GetHeight() < 5_in ) {
            move_elevator_down = false;
        }
    } else {
        // Elevator is in a place that is ok to move the elbow where it needs to go
        // if( delay_for_wrist && frc::Timer::GetFPGATimestamp() - wrist_start_time < 250_ms ) {
        if( (move_arm_backward || move_arm_forward ) && delay_for_wrist && !m_arm->WristAtGoal() ) {
            return;
        }
        if( !delay_for_elbow ) {
            if( elbow_goal > arm::kElbowRestAngle && move_arm_backward ) {
                delay_for_elbow = true;
                m_arm->SetElbowGoal( arm::kElbowRestAngle );
            } else {
                m_arm->SetElbowGoal( elbow_goal );
            }
        } else if( !move_arm_backward && !move_arm_forward ) {
            m_elevator->SetGoal( height_goal );
            if( m_elevator->GetHeight() > 5_in ) {
                delay_for_elbow = false;
            }
        }
        if( move_arm_backward ) {
            if( m_arm->GetElbowAngle() > 90_deg ) {
                move_arm_backward = false;
            }
        } else if( move_arm_forward ) {
            if( m_arm->GetElbowAngle() < 90_deg ) {
                move_arm_forward = false;
            }
        } else {
            // Arm has moved to a place where the elevator can move where it needs to go
            if( !final_move && !delay_for_elbow ) {
                m_elevator->SetGoal( height_goal );
                m_arm->SetElbowGoal( elbow_goal );
                if( wrist_goal == ArmIO::WristVertical ) {
                    if( m_arm->GetElbowAngle() < 50_deg || m_elevator->GetHeight() > 21_in ) {
                        m_arm->SetWristGoal( wrist_goal );
                        final_move = true;
                    }
                } else {
                    m_arm->SetWristGoal( wrist_goal );
                    final_move = true;
                }
            }
        }
    }
}

// Returns true when the command should end.
bool MoveMechanism::IsFinished() 
{
    bool isFinished = final_move && m_arm->AllAtGoal() && m_elevator->AtGoal();

    DataLogger::Log( "MoveMechanism/IsFinished", isFinished );

    return isFinished;
}
