
#include <ctre/phoenix6/CANBus.hpp>

#include "swerve/TalonOdometryThread.h"


TalonOdometryThread* TalonOdometryThread::singleton = nullptr;
const units::hertz_t TalonOdometryThread::CANFD_ODOMETRY_FREQUENCY = 250.0_Hz;
const units::hertz_t TalonOdometryThread::RIO_ODOMETRY_FREQUENCY = 100.0_Hz;

TalonOdometryThread* TalonOdometryThread::GetInstance() {
    if( singleton == nullptr ) {
        singleton = new TalonOdometryThread{};
    }

    return singleton;
}

std::queue<units::second_t>* TalonOdometryThread::MakeTimestampQueue() {
    odometryMutex.lock();
    std::queue<units::second_t>* tsQueue = new std::queue<units::second_t>{};
    timestampQueues.push_back( tsQueue );
    odometryMutex.unlock();

    return tsQueue;
}

std::queue<double>* TalonOdometryThread::RegisterSignal( 
    ctre::phoenix6::hardware::ParentDevice& device, 
    ctre::phoenix6::BaseStatusSignal* signal) 
{
    ctre::phoenix6::CANBus canBus( device.GetNetwork() );

    signalsMutex.lock();
    odometryMutex.lock();
    isCANFD = canBus.IsNetworkFD();
    signals.push_back( signal );
    std::queue<double>* newQ = new std::queue<double>{};
    queues.push_back( newQ );
    odometryMutex.unlock();
    signalsMutex.unlock();

    return newQ;
}

void TalonOdometryThread::Start() {
    if( timestampQueues.size() > 0 ) {
        thread = std::thread( &TalonOdometryThread::Run, this );
    }
}


void TalonOdometryThread::Run() {
    fmt::print( "       ============== TalonOdometryThread started isCANFID={} ...\n", isCANFD );

    while( 1 ) {
        signalsMutex.lock();
        if( isCANFD ) {
            ctre::phoenix6::BaseStatusSignal::WaitForAll( 2.0 / CANFD_ODOMETRY_FREQUENCY, signals );
        } else {
            std::this_thread::sleep_for( std::chrono::milliseconds( (int64_t) (1000.0 / RIO_ODOMETRY_FREQUENCY.value()) ) );
            if( signals.size() > 0 ) {
                ctre::phoenix6::BaseStatusSignal::RefreshAll( signals );
            }
        }
        signalsMutex.unlock();

        odometryMutex.lock();
        units::second_t timestamp = frc::Timer::GetFPGATimestamp();
        units::second_t totalLatency = 0_s;
        for( size_t i=0; i<signals.size(); ++i ) {
            totalLatency += signals[i]->GetTimestamp().GetLatency();
        }
        if( signals.size() > 0 ) {
            timestamp -= totalLatency / signals.size();
        }

        for( size_t i=0; i<signals.size(); ++i ) {
            if( queues[i]->size() > 10 ) {
                queues[i]->pop();
            }
            queues[i]->push( signals[i]->GetValueAsDouble() );
        }

        for( size_t i=0; i<timestampQueues.size(); ++i ) {
            if( timestampQueues[i]->size() > 10 ) {
                timestampQueues[i]->pop();
            }
            timestampQueues[i]->push( timestamp );
        }
        odometryMutex.unlock();
    }
}

