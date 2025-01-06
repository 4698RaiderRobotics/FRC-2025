// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <span>
#include <map>

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/GenericEntry.h>

#include <wpi/array.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>

class DataLogger {
  private:
      // This class is a singleton.
    static DataLogger *singleton;

      // Constructor is private
    DataLogger() {}
    static DataLogger& GetInstance();

  public:
      // delete copy constructor
    DataLogger(const DataLogger& obj) = delete; 
    static void Log( const std::string &s, double val, bool alwaysNT=false );
    static void Log( const std::string &s, std::span<const double> a, bool alwaysNT=false );
    static void Log( const std::string &s, const std::string &val, bool alwaysNT=false );
    static void Log( const std::string &s, int val, bool alwaysNT=false );
    static void Log( const std::string &s, bool val, bool alwaysNT=false );

      // WPILib Specific types
    static void Log( const std::string &s, const frc::Pose2d &p, bool alwaysNT=false );
    static void Log( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms, bool alwaysNT=false );

    static void Log( const std::string &s );
    static void LogMetadata( void );

  private:
    wpi::log::DataLog *log;
    std::shared_ptr<nt::NetworkTable> nt_table;
    std::map<std::string, nt::GenericPublisher> nt_map;

    void Send( const std::string &s, double val );
    void Send( const std::string &s, std::span<const double> a );
    void Send( const std::string &s, const std::string &val );
    void Send( const std::string &s, int val );
    void Send( const std::string &s, bool val );
    void Send( const std::string &s, const frc::Pose2d &p );
    void Send( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms );

    void SendNT( const std::string &s, double val );
    void SendNT( const std::string &s, std::span<const double> a );
    void SendNT( const std::string &s, const std::string &val );
    void SendNT( const std::string &s, int val );
    void SendNT( const std::string &s, bool val );
    void SendNT( const std::string &s, const frc::Pose2d &p );
    void SendNT( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms );

    void SendMetadata( const std::string &s, const std::string &val );

};
