
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
            m_arm->SetWristGoal( wrist_goal );
        }
        m_elevator->SetGoal( elevator::kElevatorMinHeight );
        if( m_elevator->GetHeight() < 3_in ) {
            move_elevator_down = false;
        }
    } else {
        // Elevator is in a place that is ok to move the elbow where it needs to go
        m_arm->SetElbowGoal( elbow_goal );
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
            final_move = true;
            m_elevator->SetGoal( height_goal );
            if( wrist_goal == ArmIO::WristVertical ) {
                if( m_arm->GetElbowAngle() < 50_deg || m_elevator->GetHeight() > 21_in ) {
                    m_arm->SetWristGoal( wrist_goal );
                }
            } else {
                m_arm->SetWristGoal( wrist_goal );
            }
        }
    }
}

// Returns true when the command should end.
bool MoveMechanism::IsFinished() 
{
    return final_move && m_arm->AllAtGoal() && m_elevator->AtGoal();
}
