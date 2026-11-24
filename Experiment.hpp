#pragma once

#include "ControllerChannel.hpp"
#include "ObservationChannel.hpp"
#include "PointsReader.hpp"
#include "Runtime.hpp"
#include "SessionLog.hpp"
#include "identify/ArxModel.hpp"

#include <cstdint>
#include <string>

struct ExperimentContext {
    SafeRuntime& runtime;
    SessionLog& session;
    uint64_t sessionId = 0;
    std::string sessionsRoot;
    std::string activeModelPath;
};

struct ExperimentResult {
    bool ok = false;
    std::string message;
    ArxModel model;
    double hoverThrottle = 0.0;
};

class Experiment {
public:
    virtual ~Experiment() = default;
    virtual const char* name() const = 0;
    virtual ExperimentResult run(ExperimentContext& context) = 0;
};

bool loadBaselineModel(const std::string& path, ArxModel& model, ControllerStick& hoverStick);
void saveDraftModel(const ArxModel& model, const std::string& sessionDir, const std::string& activePath, bool promote);

// Neutral sticks for a few seconds so the operator can arm / settle UI (same idea as task reset).
inline constexpr uint64_t kCalibrateResetHoldUs = 3000000;
// Returns false if interrupted (Ctrl+C).
bool runResetPhase(ExperimentContext& context, uint64_t holdUs = kCalibrateResetHoldUs);
