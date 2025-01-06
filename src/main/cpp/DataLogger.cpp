// Log data to the Data Log file on the RoboRIO
//

#include <fstream>

#include <wpi/DataLog.h>
#include <frc/Filesystem.h>
#include <frc/DataLogManager.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/StringTopic.h>

#include "DataLogger.h"


/********************************************************************************/
// Define TUNING_MODE to log everything to Network Tables which makes it much
// easier to debug things in practice.  It has the potential to overload the
// Wireless connection during competition when there is a bandwidth limit.
//
// FOR COMPETITION comment out the #define line below. 

// #define TUNING_MODE

/********************************************************************************/

#ifdef TUNING_MODE 
    #define LOG_DATA( s, val, alwaysNT ) \
       GetInstance().SendNT( s, val )
#else
    #define LOG_DATA( s, val, alwaysNT ) \
        if( alwaysNT ) { \
            GetInstance().SendNT( s, val ); \
        } \
        GetInstance().Send( s, val ); 
#endif

DataLogger* DataLogger::singleton = nullptr; 

DataLogger& DataLogger::GetInstance() {
        // If there is no instance of class
        // then we can create an instance.
    if (singleton == nullptr)  {
        singleton = new DataLogger();
        singleton->log = &frc::DataLogManager::GetLog();
        singleton->nt_table = nt::NetworkTableInstance::GetDefault().GetTable("");
    }
        
    return *singleton;
}

void DataLogger::Log( const std::string &s, double val, bool alwayNT ) { 
    LOG_DATA( s, val, alwayNT );
}

void DataLogger::Log( const std::string &s, std::span<const double> a, bool alwayNT ) { 
    LOG_DATA( s, a, alwayNT );
}

void DataLogger::Log( const std::string &s, int val, bool alwayNT ) { 
    LOG_DATA( s, val, alwayNT );
}

void DataLogger::Log( const std::string &s, const std::string &val, bool alwayNT ) { 
    LOG_DATA( s, val, alwayNT );
}

void DataLogger::Log( const std::string &s, bool val, bool alwayNT ) {
    LOG_DATA( s, val, alwayNT );
}

void DataLogger::Log( const std::string &s, const frc::Pose2d &p, bool alwayNT ) {
    LOG_DATA( s, p, alwayNT );
}

void DataLogger::Log( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms, bool alwayNT ) {
    LOG_DATA( s, sms, alwayNT );
}



void DataLogger::Send( const std::string &s, double val ) { 
    wpi::log::DoubleLogEntry le{ *log, s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, std::span<const double> a ) { 
    wpi::log::DoubleArrayLogEntry le{ *log, s };
    le.Append( a );
}

void DataLogger::Send( const std::string &s, int val ) { 
    wpi::log::IntegerLogEntry le{ *log, s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, const std::string &val ) { 
    wpi::log::StringLogEntry le{ *log, s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, bool val ) {
    wpi::log::BooleanLogEntry le{ *log, s };
    le.Append( val );
}

void DataLogger::Send( const std::string &s, const frc::Pose2d &p ) {
    static double a[3];

    wpi::log::DoubleArrayLogEntry le{ *log, s };

    a[0] = p.X().value();
    a[1] = p.Y().value();
    a[2] = p.Rotation().Radians().value();

    le.Append( a );
}

void DataLogger::Send( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms ) {
    static double a[8];

    wpi::log::DoubleArrayLogEntry le{ *log, s };

    for( int i=0; i<4; ++i ) {
        a[2*i] = sms[i].angle.Radians().value(); 
        a[2*i + 1] = sms[i].speed.value();
    }

    le.Append( a );
}


void DataLogger::SendNT( const std::string &s, double val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetDoubleTopic( s ).GenericPublish( "double" );
    }
    nt_map[s].SetDouble( val );
}

void DataLogger::SendNT( const std::string &s, std::span<const double> a ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetDoubleArrayTopic( s ).GenericPublish( "double[]" );
    }
    nt_map[s].SetDoubleArray( a );
}

void DataLogger::SendNT( const std::string &s, const std::string &val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetStringTopic( s ).GenericPublish( "string" );
    }
    nt_map[s].SetString( val );
}

void DataLogger::SendNT( const std::string &s, int val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetIntegerTopic( s ).GenericPublish( "integer" );
    }
    nt_map[s].SetInteger( val );
}

void DataLogger::SendNT( const std::string &s, bool val ) {
    if( !nt_map.contains( s ) ) {
        nt_map[s] = nt_table->GetBooleanTopic( s ).GenericPublish( "boolean" );
    }
    nt_map[s].SetBoolean( val );
}

void DataLogger::SendNT( const std::string &s, const frc::Pose2d &p ) {
    static double a[3];
    
    a[0] = p.X().value();
    a[1] = p.Y().value();
    a[2] = p.Rotation().Radians().value();

    SendNT( s, std::span{a} );
}

void DataLogger::SendNT( const std::string &s, const wpi::array<frc::SwerveModuleState, 4U> &sms ) {
    static double a[8];

    for( int i=0; i<4; ++i ) {
        a[2*i] = sms[i].angle.Radians().value(); 
        a[2*i + 1] = sms[i].speed.value();
    }
    
    SendNT( s, std::span{a} );
}


/**
 * Pass thru to the frc::DataLogManager::Log() command.
*/
void DataLogger::Log( const std::string &s ) {
    frc::DataLogManager::Log( s );
}

void DataLogger::LogMetadata( void ) {
        // Open the buildinfo.txt file and write the Metadata to the log file
    std::ifstream binfo;
    char line[256];

    std::string path = frc::filesystem::GetDeployDirectory() + "/buildinfo.txt";

    binfo.open( path, std::ios::in );
    if( binfo.is_open() ) {
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "BUILD_DATE", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_REPO", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_BRANCH", line );
        binfo.getline( line, 255 );
        GetInstance().SendMetadata( "GIT_VERSION", line );
        binfo.close();
    } else {
        Log( "Cannot open METADATA file: " + path );
    }

}

void DataLogger::SendMetadata( const std::string &s, const std::string &val ) {
        // AdvantageScope Chops off leading Character of the name so we add an underscore.
        // Not sure why
    std::string id = "RealMetadata/_";
    id += s;
    wpi::log::StringLogEntry le{ *log, id };
    le.Append( val );
}
