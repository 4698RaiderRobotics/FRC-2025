// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <units/math.h>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/Filesystem.h>

#include <frc/geometry/Twist2d.h>

#include <frc/smartdashboard/SmartDashboard.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>

#include "util/DataLogger.h"
#include "swerve/Drive.h"
#include "swerve/GyroIOPigeon2.h"
#include "swerve/ModuleIOSim.h"
#include "swerve/ModuleIOTalonFX.h"
#include "swerve/TalonOdometryThread.h"
#include "swerve/SwerveConstants.h"


Drive::Drive( ) :     
    m_kinematics{ frc::Translation2d{+( swerve::physical::kDriveBaseLength / 2 ), +( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{+( swerve::physical::kDriveBaseLength / 2 ), -( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( swerve::physical::kDriveBaseLength / 2 ), +( swerve::physical::kDriveBaseWidth / 2 )},
                    frc::Translation2d{-( swerve::physical::kDriveBaseLength / 2 ), -( swerve::physical::kDriveBaseWidth / 2 )} },
    m_odometry{ m_kinematics, rawGyroRotation, lastModulePositions, frc::Pose2d{} }
{
    SetName( "Swerve");
    
    ModuleIO *flModuleIO;
    ModuleIO *frModuleIO;
    ModuleIO *blModuleIO;
    ModuleIO *brModuleIO;
    
    if( frc::RobotBase::IsReal() ) {
        m_gyro = std::unique_ptr<GyroIOPigeon2>( new GyroIOPigeon2( swerve::pidf::pigeon2Id, swerve::pidf::swerveCanBus ) );
        flModuleIO = new ModuleIOTalonFX( swerve::pidf::flconfig );
        frModuleIO = new ModuleIOTalonFX( swerve::pidf::frconfig );
        blModuleIO = new ModuleIOTalonFX( swerve::pidf::blconfig );
        brModuleIO = new ModuleIOTalonFX( swerve::pidf::brconfig );
    } else {
        m_gyro = std::unique_ptr<GyroIO>( new GyroIO() ); 
        flModuleIO = new ModuleIOSim( swerve::pidf::flconfig );
        frModuleIO = new ModuleIOSim( swerve::pidf::frconfig );
        blModuleIO = new ModuleIOSim( swerve::pidf::blconfig );
        brModuleIO = new ModuleIOSim( swerve::pidf::brconfig );
    }

    m_modules[0] = std::unique_ptr<Module>( new Module( flModuleIO ) );
    m_modules[1] = std::unique_ptr<Module>( new Module( frModuleIO ) );
    m_modules[2] = std::unique_ptr<Module>( new Module( blModuleIO ) );
    m_modules[3] = std::unique_ptr<Module>( new Module( brModuleIO ) );

    TalonOdometryThread::GetInstance()->Start();
    
    frc::SmartDashboard::PutData("Field", &m_field);

    pathplanner::RobotConfig config;
    try {
        config = pathplanner::RobotConfig::fromGUISettings();
    } catch(const std::exception& e) {
        fmt::print( "\n\n================> PathPlanner Robot Config NOT FOUND <==================\n" );
        fmt::print( "{}\n", e.what() );
        fmt::print( "================> PathPlanner Robot Config NOT FOUND <==================\n\n" );
        m_missing_PP_config = true;

        config = pathplanner::RobotConfig( 
            70_kg, 
            6.8_kg_sq_m, 
            pathplanner::ModuleConfig( 4_in, 5.4_mps, 1.2, frc::DCMotor::KrakenX60(), 80_A, 1 ),
            swerve::physical::kDriveBaseWidth
        );
    }

    pathplanner::AutoBuilder::configure(
        [this](){ return GetPose(); },
        [this](frc::Pose2d pose){ SetPose(pose); },
        [this](){return m_kinematics.ToChassisSpeeds(GetModuleStates());},
        [this](frc::ChassisSpeeds speeds){ RunVelocity(speeds); },
        std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicDriveController, this should likely live in your Constants class
            pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            pathplanner::PIDConstants(4.0, 0.0, 0.0)  // Rotation PID constants
        ),
        config, // The robot configuration
        []() {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
        },
        this
    );
    pathplanner::PathPlannerLogging::setLogActivePathCallback( 
        [this] (std::vector<frc::Pose2d> pv) { DataLogger::Log( "Odometry/Trajectory", pv ); } );
    pathplanner::PathPlannerLogging::setLogTargetPoseCallback( 
        [this] (frc::Pose2d p) { DataLogger::Log( "Odometry/TrajectorySetpoint", p ); } );

    sysId = std::unique_ptr<frc2::sysid::SysIdRoutine>( new frc2::sysid::SysIdRoutine{ 
        frc2::sysid::Config{ std::nullopt, std::nullopt, std::nullopt, nullptr },
        frc2::sysid::Mechanism { 
            [this] (units::volt_t volts) { 
                for( int i=0; i<4; ++i ) {
                    m_modules[i]->RunCharacterization( volts );
                }
            },
            nullptr,
            this
        }
    } );
}

void Drive::RunVelocity( frc::ChassisSpeeds speeds ) {

    frc::ChassisSpeeds discreteSpeeds = frc::ChassisSpeeds::Discretize( speeds, 20_ms );

    wpi::array<frc::SwerveModuleState,4> desiredStates = m_kinematics.ToSwerveModuleStates( discreteSpeeds );

    m_kinematics.DesaturateWheelSpeeds( &desiredStates, swerve::physical::kMaxDriveSpeed );

    wpi::array<frc::SwerveModuleState,4> optimizedStates{wpi::empty_array};
    for( int i=0; i<4; ++i ) {
        optimizedStates[i] = m_modules[i]->RunSetpoint( desiredStates[i] );
    }

    DataLogger::Log( "Swerve/desiredStates", desiredStates );
    DataLogger::Log( "Swerve/optimizedStates", optimizedStates );
}

void Drive::Periodic( void ) {
        // Set PathPlanner config alert condition
    PPalert.Set( m_missing_PP_config );

        // Get new input values
    TalonOdometryThread::GetInstance()->odometryMutex.lock();
    m_gyro->UpdateInputs( gyroInputs );
    for( int i=0; i<4; ++i ) {
        m_modules[i]->UpdateInputs();
    }
    TalonOdometryThread::GetInstance()->odometryMutex.unlock();

        // Log new input values
    gyroInputs.LogInputs( "Swerve/Gyro" );
    for( int i=0; i<4; ++i ) {
        m_modules[i]->Periodic();
    }

    // if(frc::DriverStation::IsDisabled() && !m_have_driver_offset ) {
    //     auto pose = m_odometry.GetEstimatedPosition();
    //     field_offset = pose.Rotation().Degrees();
    //     if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) {
    //         driver_offset = field_offset + 180_deg;
    //     } else {
    //         driver_offset = field_offset;
    //     }
    // } else if( !frc::DriverStation::IsDisabled() && !m_have_driver_offset ) {
    //         // Only get a driver offset on the first enabling..
    //     m_have_driver_offset = true;
    //      fmt::print( "SwerveDriveSubsystem::Periodic -- Stop getting offset has {} = {:.5}\n", m_have_driver_offset, driver_offset.value() );
    // }

    if( frc::DriverStation::IsDisabled() ) {
        for( int i=0; i<4; ++i ) {
            m_modules[i]->Stop();
        }

        DataLogger::Log( "Swerve/desiredStates", std::span<frc::SwerveModuleState>{} );
        DataLogger::Log( "Swerve/optimizedStates", std::span<frc::SwerveModuleState>{} );
    }

        // Update odometry
    const std::vector<units::second_t>& sampleTimestamps = m_modules[0]->getOdometryTimestamps();
    size_t sampleCount = sampleTimestamps.size();
    wpi::array<frc::SwerveModulePosition,4U> modulePositions{wpi::empty_array};
    wpi::array<frc::SwerveModulePosition,4U> moduleDeltas{wpi::empty_array};
    for( size_t i=0; i<sampleCount; ++i ) {
        for( int moduleIndex=0; moduleIndex<4; ++ moduleIndex ) {
            modulePositions[moduleIndex] = m_modules[moduleIndex]->getOdometryPositions()[i];
            if( !gyroInputs.connected ) {
                moduleDeltas[moduleIndex] = {
                    modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                    modulePositions[moduleIndex].angle };
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }
        }

        if( gyroInputs.connected ) {
            rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } else {
            frc::Twist2d twist = m_kinematics.ToTwist2d(moduleDeltas);
            rawGyroRotation += twist.dtheta;
        }

        m_odometry.UpdateWithTime( sampleTimestamps[i], rawGyroRotation, modulePositions );
    }

    DataLogger::Log( "Swerve/actualStates",  GetModuleStates() );
    DataLogger::Log( "Swerve/Pose2d", m_odometry.GetEstimatedPosition() );
    m_field.SetRobotPose( m_odometry.GetEstimatedPosition() );
}

void Drive::SetWheelAngles( std::vector<units::radian_t> angles ) {
    for( int i=0; i<4; ++i ) {
        m_modules[i]->RunSetpoint( { 0_mps, angles[i] } );
    }
}

void Drive::SetDriveVelocity( units::meters_per_second_t vel ) {
    for( int i=0; i<4; ++i ) {
        m_modules[i]->RunSetpoint( { vel, m_modules[i]->GetAngle() } );
    }
}

void Drive::Stop() 
{
    RunVelocity( {0_mps, 0_mps, 0_rad_per_s} );
    for( int i=0; i<4; ++i ) {
        m_modules[i]->Stop();
    }
}

wpi::array<frc::SwerveModuleState,4U>& Drive::GetModuleStates() 
{
    static wpi::array<frc::SwerveModuleState,4U> states{wpi::empty_array};
    for( int i=0; i<4; ++i ) {
        states[i] = m_modules[i]->GetState();
    }
    return states;
}

// Returns the pose2d of the robot
frc::Pose2d Drive::GetPose( void ) {
    return m_odometry.GetEstimatedPosition();
}

// Returns the rotation of the robot
frc::Rotation2d Drive::GetRotation( void ) 
{
    return GetPose().Rotation();
}

frc::ChassisSpeeds Drive::GetChassisSpeeds()
{
    return m_kinematics.ToChassisSpeeds( GetModuleStates() );
}


// Resets the gyro to an angle
void Drive::ResetGyro() {
    DataLogger::Log( "Swerve/Status", "Reseting Gyro.." );
    m_gyro->Reset();
}

// Resets the pose to a position
void Drive::SetPose( frc::Pose2d pose ) {
    DataLogger::Log( "Swerve/Status", fmt::format("Reseting Pose to <{},{},{}> with GyroYaw {}..", 
                    pose.X(), pose.Y(), pose.Rotation().Degrees(), rawGyroRotation ) );
    m_odometry.ResetPosition(
        rawGyroRotation,
        {
            m_modules[0]->GetPosition(),  m_modules[1]->GetPosition(), 
            m_modules[2]->GetPosition(),  m_modules[3]->GetPosition() 
        },
        pose
    );
}

void GyroIO::Inputs::LogInputs( std::string key ) {
    AUTOLOG( key, connected )
    AUTOLOG( key, yawPosition )
    AUTOLOG( key, yawVelocity )

    AUTOLOG( key, odometryYawTimestamps )
    AUTOLOG( key, odometryYawPositions )
}

