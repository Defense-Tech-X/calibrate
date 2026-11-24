#include "Experiment.hpp"
#include "Runtime.hpp"

#include <cstdio>

#include <windows.h>

bool loadBaselineModel(const std::string& path, ArxModel& model, ControllerStick& hoverStick) {
    if (!model.load(path)) {
        return false;
    }
    double throttle = model.baseline()[0];
    if (model.sticks().present) {
        if (model.sticks().hoverMeasured && model.sticks().hoverThrottle > 0.0) {
            throttle = model.sticks().hoverThrottle;
        } else if (model.sticks().liftoffThrottle > 0.0) {
            throttle = model.sticks().liftoffThrottle;
        }
    }
    hoverStick.throttle = static_cast<uint16_t>(throttle);
    hoverStick.yaw = static_cast<uint16_t>(model.baseline()[1]);
    hoverStick.pitch = static_cast<uint16_t>(model.baseline()[2]);
    hoverStick.roll = static_cast<uint16_t>(model.baseline()[3]);
    hoverStick.flightMode = kFlightModeAngle;
    return true;
}

void saveDraftModel(
    const ArxModel& model,
    const std::string& sessionDir,
    const std::string& activePath,
    bool promote) {
    model.save(sessionDir + "/model.json");
    if (promote) {
        CreateDirectoryA("plant", nullptr);
        CreateDirectoryA("plant/models", nullptr);
        model.save(activePath);
        std::printf("wrote active model: %s\n", activePath.c_str());
    }
}

bool runResetPhase(ExperimentContext& context, uint64_t holdUs) {
    std::printf(
        "reset: thr=0 sticks centered for %.1fs (arm craft / prepare UI)\n",
        holdUs / 1.0e6);
    std::fflush(stdout);

    uint64_t phaseStartUs = 0;
    uint64_t lastLogUs = 0;
    bool interrupted = false;

    auto tick = [&](uint64_t nowUs,
                    const ControllerStick&,
                    const PointCloud*,
                    const PointShift*,
                    const ObservationSample*,
                    bool) -> bool {
        if (context.runtime.stopped()) {
            interrupted = true;
            return false;
        }
        if (phaseStartUs == 0) {
            phaseStartUs = nowUs;
        }
        ControllerStick neutral;
        context.runtime.writeStick(neutral);
        if (lastLogUs == 0 || nowUs - lastLogUs >= 1000000) {
            lastLogUs = nowUs;
            const double left = holdUs > (nowUs - phaseStartUs)
                ? (holdUs - (nowUs - phaseStartUs)) / 1.0e6
                : 0.0;
            std::printf("reset: holding neutral, %.1fs left\n", left);
            std::fflush(stdout);
        }
        return nowUs - phaseStartUs < holdUs;
    };

    context.runtime.runLoop(tick, nullptr, "reset", context.sessionId);
    if (interrupted || context.runtime.stopped()) {
        std::printf("reset: interrupted\n");
        std::fflush(stdout);
        return false;
    }
    std::printf("reset: done, starting experiment\n");
    std::fflush(stdout);
    return true;
}
