// Log data to the Data Log file on the RoboRIO
//

#include <fstream>

#include <wpi/DataLog.h>
#include <frc/Filesystem.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/geometry/Pose2d.h>
#include <networktables/DoubleTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/BooleanArrayTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/GenericEntry.h>

#include "util/DataLogger.h"

DataLogger* DataLogger::singleton = nullptr;

DataLogger& DataLogger::GetInstance() {
    // If there is no instance of class
    // then we can create an instance.
    if (singleton == nullptr)  {
        singleton = new DataLogger();
        singleton->log = &frc::DataLogManager::GetLog();
        singleton->nt_table = nt::NetworkTableInstance::GetDefault().GetTable("");
        singleton->isFMSAttached = frc::DriverStation::IsFMSAttached();
        Log( "DataLogger/isFMSAttahed", singleton->isFMSAttached );
    }
        
    return *singleton;
}

void DataLogger::Log( const std::string& s, const std::string& val ) 
{
    if( GetInstance().isFMSAttached ) { 
        GetInstance().Send(s,val); 
    } else { 
        GetInstance().SendNT(s,val); 
    }
 }

// Specialization for a std::optional<Pose2d>
template<>
void DataLogger::Log( const std::string &s, const std::optional<frc::Pose2d>& opt ) {
    if( opt.has_value() ) {
        std::vector<frc::Pose2d> v{ opt.value() };
        DataLogger::Log( s, v );
    } else {
        DataLogger::Log( s, std::span<frc::Pose2d>{} );
    }
}

/**
 * Pass thru to the frc::DataLogManager::Log() command.
*/
void DataLogger::Log( const std::string &s ) {
    frc::DataLogManager::Log( s );
}


void DataLogger::Send( const std::string& s, const double& val )
{
    wpi::log::DoubleLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, const int64_t& val )
{
    wpi::log::IntegerLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, const bool& val )
{
    wpi::log::BooleanLogEntry le{ *(log), s };
    le.Append( val );
}

void DataLogger::Send( const std::string& s, const std::string& val ) { 
    wpi::log::StringLogEntry le{ *(log), s };
    le.Append( val );
}


void DataLogger::SendNT( const std::string& s, const double& val )
{
    nt::DoublePublisher* publisher;
    std::unordered_map<std::string, nt::Publisher*>::iterator i;

    i = nt_map.find( s );
    if( i == nt_map.end() ) {
        publisher = new nt::DoublePublisher();
        *publisher = nt_table->GetDoubleTopic( s ).Publish( );
        i = nt_map.insert(std::make_pair(s, publisher)).first;
    }
    publisher = (nt::DoublePublisher*) i->second;
    publisher->Set( val );
}

void DataLogger::SendNT( const std::string& s, const int64_t& val )
{
    nt::IntegerPublisher* publisher;
    std::unordered_map<std::string, nt::Publisher*>::iterator i;

    i = nt_map.find( s );
    if( i == nt_map.end() ) {
        publisher = new nt::IntegerPublisher();
        *publisher = nt_table->GetIntegerTopic( s ).Publish( );
        i = nt_map.insert(std::make_pair(s, publisher)).first;
    }
    publisher = (nt::IntegerPublisher*) i->second;
    publisher->Set( val );
}

void DataLogger::SendNT( const std::string& s, const bool& val )
{
    nt::BooleanPublisher* publisher;
    std::unordered_map<std::string, nt::Publisher*>::iterator i;

    i = nt_map.find( s );
    if( i == nt_map.end() ) {
        publisher = new nt::BooleanPublisher();
        *publisher = nt_table->GetBooleanTopic( s ).Publish( );
        i = nt_map.insert(std::make_pair(s, publisher)).first;
    }
    publisher = (nt::BooleanPublisher*) i->second;
    publisher->Set( val );
}

void DataLogger::SendNT( const std::string &s, const std::string &val ) 
{
    nt::StringPublisher* publisher;
    std::unordered_map<std::string, nt::Publisher*>::iterator i;

    i = nt_map.find( s );
    if( i == nt_map.end() ) {
        publisher = new nt::StringPublisher();
        *publisher = nt_table->GetStringTopic( s ).Publish( );
        i = nt_map.insert(std::make_pair(s, publisher)).first;
    }
    publisher = (nt::StringPublisher*) i->second;
    publisher->Set( val );
}


void DataLogger::LogMetadata( void ) {
        // Open the buildinfo.txt file and write the Metadata to the log file
    std::ifstream binfo;
    std::string fname;
    char line[256];

    fname = frc::filesystem::GetDeployDirectory() + "/buildinfo.txt";

    binfo.open( fname, std::ios::in );
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
        Log( "Cannot open Metadata file: " + fname );
    }
}

void DataLogger::SendMetadata( std::string_view s, std::string_view val ) {
        // AdvantageScope Chops off leading Character of the name so we add an underscore.
        // Not sure why
    std::string id = "RealMetadata/_";
    id += s;
    wpi::log::StringLogEntry le{ *(log), id };
    le.Append( val );
}
