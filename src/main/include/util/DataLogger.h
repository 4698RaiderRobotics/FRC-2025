// Log data to the Data Log file on the RoboRIO
//

#pragma once

#include <span>
#include <unordered_map>
#include <units/base.h>
#include <units/math.h>
#include <wpi/array.h>

#include <wpi/DataLog.h>
#include <frc/DataLogManager.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StructTopic.h>
#include <networktables/StructArrayTopic.h>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/kinematics/SwerveModulePosition.h>

/*
 *  AUTOLOG macro can log the following data types:
 *      intrinsics, vectors of intrinsics, units, vectors of units, StructSerializable,
 *      std::optional<> wrapped types
 * 
 *  It uses the variable name as the field name and will add the units abbreviation for unit types.  
 */
#define AUTOLOG(key,v) DataLogger::Log( std::string{key} + "/" + #v, v );

class DataLogger {
private:
    // This class is a singleton.
    static DataLogger *singleton;

    // Constructor is private
    DataLogger();
    static DataLogger* GetInstance();

public:

    // delete copy constructor
    DataLogger(const DataLogger& obj) = delete; 

    static void Log( const std::string& s, const double& val );
    static void Log( const std::string& s, const int64_t& val );
    static void Log( const std::string& s, const int& val );
    static void Log( const std::string& s, const bool& val );
    static void Log( const std::string& s, const std::string& val );
    static void Log( const std::string& s, const char* val );


    template <class T>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, const T& val );
    template <class T>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, std::span<T> a );
    template <class T>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, std::vector<T>& v );
    template <class T, size_t N>
     requires wpi::StructSerializable<T>
    static void Log( const std::string& s, wpi::array<T,N>& a );


        // A units library type.
    template <class UnitType>
    requires units::traits::is_unit_t<UnitType>::value
    static void Log( const std::string& s, const UnitType& val ) noexcept;

        // A vector of units library type.
    template <class UnitType>
    requires units::traits::is_unit_t<UnitType>::value
    static void Log( const std::string& s, const std::vector<UnitType>& vec ) noexcept;

        // A std::optional wrapped type.
    template<class T>
    static void Log( const std::string& s, const std::optional<T>& opt );

    static void LogMetadata( void );

private:
    std::shared_ptr<nt::NetworkTable> nt_table;
    std::unordered_map<std::string, nt::Publisher*> nt_map;
};

template <class UnitType>
requires units::traits::is_unit_t<UnitType>::value
void DataLogger::Log( const std::string &s, const UnitType& val ) noexcept {
    DataLogger::Log( s + "(" + units::abbreviation(val) + ")", val.value() );
}

template <class UnitType>
requires units::traits::is_unit_t<UnitType>::value
void DataLogger::Log( const std::string &s, const std::vector<UnitType>& vec ) noexcept 
{
    static std::vector<double> a;
    
    a.clear();
    for( size_t i=0; i<vec.size(); ++i ) {
        a.push_back(vec[i].value());
    }
    DataLogger::Log( s + "(" + units::abbreviation(UnitType(0)) + ")", std::span<double>( a ) );
}

template<class T>
void DataLogger::Log( const std::string &s, const std::optional<T>& opt ) 
{
    if( opt.has_value() ) {
        DataLogger::Log( s, opt.value() );
    }
}

        // frc::Pose2d specialization
template<> void DataLogger::Log( const std::string &s, const std::optional<frc::Pose2d>& opt );

template <class T>
 requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, std::vector<T>& v )
{
    Log( s, std::span<T>(v) );
}

template <class T, size_t N>
 requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, wpi::array<T,N>& a )
{
    Log( s, std::span<T>(a) );
}

template <class T>
requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, const T& val ) 
{
    nt::StructPublisher<T>* publisher;
    std::unordered_map<std::string, nt::Publisher*>::iterator i;
    DataLogger *dl = GetInstance();

    i = dl->nt_map.find( s );
    if( i == dl->nt_map.end() ) {
        publisher = new nt::StructPublisher<T>();
        *publisher = dl->nt_table->GetStructTopic<T>( s ).Publish();
        i = dl->nt_map.insert(std::make_pair(s, publisher) ).first;
    }
    publisher = (nt::StructPublisher<T>*) i->second;
    publisher->Set( val );
}

template <class T>
 requires wpi::StructSerializable<T>
void DataLogger::Log( const std::string& s, std::span<T> a ) 
{
    nt::StructArrayPublisher<T>* publisher;
    std::unordered_map<std::string, nt::Publisher*>::iterator i;
    DataLogger *dl = GetInstance();

    i = dl->nt_map.find( s );
    if( i == dl->nt_map.end() ) {
        publisher = new nt::StructArrayPublisher<T>();
        *publisher = dl->nt_table->GetStructArrayTopic<T>( s ).Publish();
        i = dl->nt_map.insert(std::make_pair(s, publisher) ).first;
    }
    publisher = (nt::StructArrayPublisher<T>*) i->second;
    publisher->Set( a );
}
