#pragma once

#include "ControllerChannel.hpp"
#include "ObservationChannel.hpp"
#include "PointsReader.hpp"
#include "SessionLog.hpp"
#include "common/Observation.hpp"
#include "motion/PointShift.hpp"

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

uint64_t qpcNowUs();

class SafeRuntime {
public:
    SafeRuntime();
    ~SafeRuntime();

    void setupVisionAndController();
    void shutdown();
    bool stopped() const { return stop_.load(); }
    void requestStop() { stop_.store(true); }

    ControllerChannel& controller() { return controller_; }
    PointsReader& points() { return points_; }
    ObservationChannel& observation() { return observation_; }

    bool readStick(ControllerStick& stick);
    bool writeStick(const ControllerStick& stick);
    void parkThrottle();

    using TickFn = std::function<bool(
        uint64_t nowUs,
        const ControllerStick& current,
        const PointCloud* cloud,
        const PointShift* sincePrevious,
        const ObservationSample* observation,
        bool referenceReady)>;

    void runLoop(TickFn tick, SessionLog* session, const char* phase, uint64_t sessionId);

private:
    std::mutex io_;
    std::atomic<bool> stop_{ false };
    bool parked_ = false;
    ControllerChannel controller_;
    PointsReader points_;
    ObservationChannel observation_;
    std::vector<TrackedPoint> previous_;
};
