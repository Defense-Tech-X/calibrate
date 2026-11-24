#pragma once

#include "identify/VisionMotionParams.hpp"
#include "motion/PointShift.hpp"

#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

// Shared live takeoff to apex picture lock:
//   Climb(liftoff) -> Clearance(liftoff, 1s) -> Coast(stopClimb) -> lock on apex.
// Used by roll_pulse and pitch_pulse. hover_trim coasts at u_eq until speed
// crosses zero, then raises kH with no ceiling. Reset stays in ExperimentUtils.
namespace ApexClimb {

inline constexpr uint64_t kClearanceUs = 1000000;
inline constexpr uint64_t kCoastWaitUs = 5000000;

enum class Phase {
    Climb,
    Clearance,
    Coast,
    Locked,
};

struct Config {
    const char* experimentName = "takeoff";
    uint16_t liftoff = 0;
    uint16_t stopClimb = 0;
    VisionMotionParams vision;
    uint64_t clearanceUs = kClearanceUs;
    uint64_t coastWaitUs = kCoastWaitUs;
};

struct State {
    Phase phase = Phase::Climb;
    uint64_t phaseStartUs = 0;
    uint32_t climbFrames = 0;
    uint32_t stillFrames = 0;
    bool haveClimbPrev = false;
    bool haveCoastPrev = false;
    bool sawCoastClimb = false;
    bool altitudeLocked = false;
    std::vector<TrackedPoint> climbPrev;
    std::vector<TrackedPoint> coastPrev;
    std::vector<TrackedPoint> altitudeRef;
    uint16_t throttle = 0;
    std::string lockWhy;
};

struct StepResult {
    bool ok = true;
    bool justLocked = false;
    const char* failMessage = nullptr;
};

inline bool tryLock(
    State& state,
    const PointCloud* cloud,
    const char* why) {
    if (cloud == nullptr || cloud->points.size() < kMinMatchedPoints) {
        return false;
    }
    state.altitudeRef = cloud->points;
    state.altitudeLocked = true;
    state.lockWhy = why != nullptr ? why : "apex";
    state.phase = Phase::Locked;
    return true;
}

inline void begin(State& state, const Config& cfg, uint64_t nowUs) {
    state = State{};
    state.phase = Phase::Climb;
    state.phaseStartUs = nowUs;
    state.throttle = cfg.liftoff;
    std::printf(
        "%s: climb at thr=%u until leave-ground -> clearance %.2fs -> "
        "dump stopClimb=%u -> coast apex\n",
        cfg.experimentName,
        cfg.liftoff,
        cfg.clearanceUs / 1.0e6,
        cfg.stopClimb);
    std::fflush(stdout);
}

// One control tick for Climb/Clearance/Coast. Caller writes state.throttle.
// When phase becomes Locked, justLocked=true and altitudeRef is ready.
inline StepResult step(
    State& state,
    const Config& cfg,
    uint64_t nowUs,
    const PointCloud* cloud) {
    StepResult out;
    if (state.phase == Phase::Locked) {
        return out;
    }
    if (state.phaseStartUs == 0) {
        begin(state, cfg, nowUs);
    }

    if (state.phase == Phase::Climb) {
        state.throttle = cfg.liftoff;
        if (cloud != nullptr && cloud->points.size() >= kMinMatchedPoints) {
            if (state.haveClimbPrev) {
                const FocusShift focus =
                    measureFocusShift(state.climbPrev, cloud->points, cfg.vision);
                if (focus.valid && focus.climbing) {
                    state.climbFrames += 1;
                } else if (focus.valid) {
                    state.climbFrames = 0;
                }
            }
            state.climbPrev = cloud->points;
            state.haveClimbPrev = true;
        }
        if (state.climbFrames >= cfg.vision.climbConfirmFrames) {
            state.phase = Phase::Clearance;
            state.phaseStartUs = nowUs;
            state.climbFrames = 0;
            state.haveClimbPrev = false;
            state.climbPrev.clear();
            state.altitudeLocked = false;
            state.altitudeRef.clear();
            std::printf(
                "%s: leave-ground -> clearance %.2fs then dump stopClimb=%u\n",
                cfg.experimentName,
                cfg.clearanceUs / 1.0e6,
                cfg.stopClimb);
            std::fflush(stdout);
        }
        return out;
    }

    if (state.phase == Phase::Clearance) {
        state.throttle = cfg.liftoff;
        if (nowUs - state.phaseStartUs >= cfg.clearanceUs) {
            state.throttle = cfg.stopClimb;
            state.phase = Phase::Coast;
            state.phaseStartUs = nowUs;
            state.haveCoastPrev = false;
            state.coastPrev.clear();
            state.sawCoastClimb = false;
            state.stillFrames = 0;
            state.altitudeLocked = false;
            state.altitudeRef.clear();
            std::printf(
                "%s: clearance done -> dump thr=%u, coast until apex (vz~0)\n",
                cfg.experimentName,
                state.throttle);
            std::fflush(stdout);
        }
        return out;
    }

    // Coast
    state.throttle = cfg.stopClimb;
    if (cloud != nullptr && cloud->points.size() >= kMinMatchedPoints) {
        if (state.haveCoastPrev) {
            const FocusShift focus =
                measureFocusShift(state.coastPrev, cloud->points, cfg.vision);
            if (focus.valid) {
                if (focus.climbing) {
                    state.sawCoastClimb = true;
                    state.stillFrames = 0;
                } else if (focus.descending) {
                    state.stillFrames = 0;
                    if (tryLock(
                            state,
                            cloud,
                            state.sawCoastClimb ? "apex past / descend start"
                                                : "coast descend")) {
                        out.justLocked = true;
                        std::printf(
                            "%s: HEIGHT LOCK points=%u thr=%u (%s)\n",
                            cfg.experimentName,
                            static_cast<unsigned>(state.altitudeRef.size()),
                            state.throttle,
                            state.lockWhy.c_str());
                        std::fflush(stdout);
                        return out;
                    }
                } else {
                    state.stillFrames += 1;
                    if (state.stillFrames >= cfg.vision.climbConfirmFrames) {
                        if (tryLock(
                                state,
                                cloud,
                                state.sawCoastClimb
                                    ? "apex vz~0 after coast climb"
                                    : "coast still")) {
                            out.justLocked = true;
                            std::printf(
                                "%s: HEIGHT LOCK points=%u thr=%u (%s)\n",
                                cfg.experimentName,
                                static_cast<unsigned>(state.altitudeRef.size()),
                                state.throttle,
                                state.lockWhy.c_str());
                            std::fflush(stdout);
                            return out;
                        }
                    }
                }
            }
        }
        state.coastPrev = cloud->points;
        state.haveCoastPrev = true;
    }

    if (!state.altitudeLocked && nowUs - state.phaseStartUs >= cfg.coastWaitUs) {
        if (tryLock(state, cloud, "coast timeout")) {
            out.justLocked = true;
            std::printf(
                "%s: HEIGHT LOCK points=%u thr=%u (%s)\n",
                cfg.experimentName,
                static_cast<unsigned>(state.altitudeRef.size()),
                state.throttle,
                state.lockWhy.c_str());
            std::fflush(stdout);
            return out;
        }
        out.ok = false;
        out.failMessage = "no apex lock after stopClimb dump (no points)";
        return out;
    }
    return out;
}

inline const char* phaseName(Phase phase) {
    switch (phase) {
    case Phase::Climb:
        return "climb";
    case Phase::Clearance:
        return "clearance";
    case Phase::Coast:
        return "coast";
    case Phase::Locked:
        return "locked";
    }
    return "takeoff";
}

} // namespace ApexClimb
