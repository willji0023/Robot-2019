// Copyright (c) 2016-2019 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "dsdisplay/UdpSocket.hpp"
#include "logging/CsvLogger.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train
 */
class Vision : public SubsystemBase, public PublishNode {
public:
    struct PnP {
    };

    Vision();
    ~Vision();

    void SubsystemPeriodic() override;

private:
    std::thread m_thread;
    std::atomic<bool> m_isRunning{true};
    std::mutex m_mutex;

    UdpSocket m_socket;
    uint32_t m_rpiIP = kRpiIP;
    uint16_t m_port = kRpiPort;
    PnP m_pnp;

    CsvLogger visionLogger{"/home/lvuser/Vision.csv",
                           "Time,Rotation0,Rotation1,Rotation2,Translation0,"
                           "Translation1,Translation2"};

    void RetrieveMatrices();
};
}  // namespace frc3512
