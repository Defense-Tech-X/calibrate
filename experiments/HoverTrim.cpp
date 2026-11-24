#include "experiments/HoverTrim.hpp"

#include "Experiment.hpp"
#include "Runtime.hpp"
#include "hold/AltitudeHold.hpp"
#include "identify/AltitudePHold.hpp"
#include "motion/PointShift.hpp"
#include "takeoff/ApexClimb.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

namespace {
// Restored cascade gains from the stable hold run (before Kv/vzMax softening).
inline constexpr float kHoldKpAlt = 1.0f;
inline constexpr float kHoldKv = 55.0f;
inline constexpr float kHoldVzMax = 3.0f;
inline constexpr float kHoverKi = 25.0f;
inline constexpr float kStillDyPx = 0.8f;
inline constexpr float kStillVzPxPerSec = 1.2f;
// I on hover only near lock - far excursions must not walk hover to stopClimb.
inline constexpr float kHoverIntegrateDyPx = 2.0f;
// Tolerate brief lock noise without resetting the 3s quiet timer.
inline constexpr uint64_t kStillBreakGraceUs = 250000;
} // namespace

const char* HoverTrimExperiment::name() const {
    return "hover_trim";
}

ExperimentResult HoverTrimExperiment::run(ExperimentContext& context) {
    ExperimentResult result;
    ArxModel prior;
    ControllerStick stick;
    if (!loadBaselineModel(context.activeModelPath, prior, stick)) {
        result.message = "active model required; run throttle_step first";
        return result;
    }

    VisionMotionParams vision = prior.visionMotion();
    if (!vision.present) {
        result.message = "visionMotion missing; run throttle_step first";
        return result;
    }
    if (vision.climbConfirmFrames < 3) {
        vision.climbConfirmFrames = 3;
    }
    vision.hoverHoldDyPx = kStillDyPx;
    vision.hoverHoldRatePxPerSec = kStillVzPxPerSec;
    vision.holdAltDeadband = vision.hoverHoldDyPx;
    vision.hoverVerifySec = 3.0f;
    vision.holdKpAlt = kHoldKpAlt;
    vision.holdKv = kHoldKv;
    vision.holdVzMax = kHoldVzMax;
    vision.holdThrPerPx = AltitudePHold::kThrPerPxDefault;

    StickCalibration sticks = prior.sticks();
    const uint16_t liftoff = static_cast<uint16_t>(
        sticks.liftoffThrottle > 0.0 ? sticks.liftoffThrottle : 0.0);
    uint16_t stopClimb = 0;
    if (sticks.stopClimbThrottle > 0.0) {
        stopClimb = static_cast<uint16_t>(sticks.stopClimbThrottle);
    } else if (sticks.hoverThrottle > 0.0) {
        stopClimb = static_cast<uint16_t>(sticks.hoverThrottle);
    }
    if (liftoff == 0 || stopClimb == 0) {
        result.message = "liftoffThrottle + stopClimbThrottle required; run throttle_step first";
        return result;
    }
    if (liftoff <= stopClimb) {
        result.message = "liftoffThrottle must be above stopClimbThrottle";
        return result;
    }

    if (!runResetPhase(context)) {
        result.message = "interrupted during reset";
        return result;
    }

    context.session.open(context.sessionId, context.sessionsRoot, "hover_trim");

    enum class Phase { Takeoff, Hold, Done };
    Phase phase = Phase::Takeoff;

    const uint64_t regIntervalUs = 100000;
    const uint64_t logEveryUs = 500000;
    const uint64_t stillNeedUs = 5000000;
    const float holdDyBand = vision.hoverHoldDyPx;
    const float rateBand = vision.hoverHoldRatePxPerSec;
    const int slewMax = AltitudePHold::kSlewMax;
    const int authority = static_cast<int>(liftoff) - static_cast<int>(stopClimb);
    // Start hover between stopClimb and liftoff; I-term refines it on lock.
    const uint16_t hoverSeed = static_cast<uint16_t>(
        static_cast<int>(stopClimb) + authority * 2 / 5);

    uint16_t hoverBase = hoverSeed;
    uint16_t hoverTrim = hoverSeed;
    uint16_t throttle = liftoff;
    uint64_t phaseStartUs = 0;
    uint64_t lastLogUs = 0;
    uint64_t lastRegUs = 0;
    uint64_t stillAccumUs = 0;
    uint64_t stillSampleUs = 0;
    uint64_t stillBreakSinceUs = 0;
    uint64_t prevDyUs = 0;
    float lastDy = 0.0f;
    float prevDy = 0.0f;
    float lastVz = 0.0f;
    bool havePrevDy = false;
    bool altitudeLocked = false;
    double stillThrSum = 0.0;
    uint32_t stillThrSamples = 0;

    std::vector<TrackedPoint> altitudeRef;
    ApexClimb::Config takeoffCfg;
    takeoffCfg.experimentName = "hover_trim";
    takeoffCfg.liftoff = liftoff;
    takeoffCfg.stopClimb = stopClimb;
    takeoffCfg.vision = vision;
    ApexClimb::State takeoff;

    std::printf(
        "hover_trim: ApexClimb -> clearance %.2fs -> dump stopClimb -> apex LOCK\n"
        "hover_trim: cascade vzCmd=-Kp*dy, thr=hover-Kv*(vz-vzCmd); "
        "liftoff=%u stopClimb=%u seed=%u Kp=%.2f Kv=%.1f vzMax=%.1f Ki=%.1f "
        "db=%.2f still=%.1fs\n"
        "hover_trim: session=%s\n",
        takeoffCfg.clearanceUs / 1.0e6,
        liftoff,
        stopClimb,
        hoverSeed,
        vision.holdKpAlt,
        vision.holdKv,
        vision.holdVzMax,
        kHoverKi,
        holdDyBand,
        stillNeedUs / 1.0e6,
        context.session.directory().c_str());
    std::fflush(stdout);

    auto writeStick = [&](uint16_t thr) {
        ControllerStick command;
        command.throttle = thr;
        command.yaw = kAxisCenter;
        command.pitch = kAxisCenter;
        command.roll = kAxisCenter;
        command.flightMode = kFlightModeAngle;
        context.runtime.writeStick(command);
    };

    auto tick = [&](uint64_t nowUs,
                    const ControllerStick&,
                    const PointCloud* cloud,
                    const PointShift*,
                    const ObservationSample*,
                    bool) -> bool {
        if (context.runtime.stopped()) {
            result.message = "interrupted";
            return false;
        }
        if (phaseStartUs == 0) {
            phaseStartUs = nowUs;
            ApexClimb::begin(takeoff, takeoffCfg, nowUs);
        }

        if (phase == Phase::Takeoff) {
            const ApexClimb::StepResult takeoffStep =
                ApexClimb::step(takeoff, takeoffCfg, nowUs, cloud);
            throttle = takeoff.throttle;
            if (!takeoffStep.ok) {
                result.message = takeoffStep.failMessage != nullptr
                    ? takeoffStep.failMessage
                    : "apex takeoff failed";
                return false;
            }
            if (takeoffStep.justLocked || takeoff.phase == ApexClimb::Phase::Locked) {
                altitudeRef = takeoff.altitudeRef;
                altitudeLocked = true;
                // Start at seed, not stopClimb: avoids +100 slew pumping climb at apex.
                throttle = hoverSeed;
                hoverTrim = hoverSeed;
                hoverBase = hoverSeed;
                phase = Phase::Hold;
                phaseStartUs = nowUs;
                lastRegUs = 0;
                stillAccumUs = 0;
                stillSampleUs = 0;
                stillBreakSinceUs = 0;
                stillThrSum = 0.0;
                stillThrSamples = 0;
                lastDy = 0.0f;
                prevDy = 0.0f;
                lastVz = 0.0f;
                havePrevDy = false;
                prevDyUs = 0;
                std::printf(
                    "hover_trim: HOLD cascade around seed=%u "
                    "(stopClimb=%u liftoff=%u Kp=%.2f Kv=%.1f vzMax=%.1f); "
                    "still need %.1fs\n",
                    hoverTrim,
                    stopClimb,
                    liftoff,
                    vision.holdKpAlt,
                    vision.holdKv,
                    vision.holdVzMax,
                    stillNeedUs / 1.0e6);
                std::fflush(stdout);
            }
        } else if (phase == Phase::Hold) {
            if (!altitudeLocked) {
                result.message = "hold without height lock";
                return false;
            }

            float dtReg = static_cast<float>(regIntervalUs) * 1.0e-6f;
            if (cloud != nullptr && cloud->points.size() >= kMinMatchedPoints) {
                const LockError lock = measureLockError(
                    altitudeRef, cloud->points, kMinMatchedPoints);
                if (lock.valid) {
                    if (havePrevDy && prevDyUs > 0 && nowUs > prevDyUs) {
                        const float dtSec =
                            static_cast<float>(nowUs - prevDyUs) * 1.0e-6f;
                        if (dtSec > 1.0e-4f) {
                            lastVz = (lock.medianDy - prevDy) / dtSec;
                            dtReg = dtSec;
                        }
                    }
                    prevDy = lock.medianDy;
                    prevDyUs = nowUs;
                    havePrevDy = true;
                    lastDy = lock.medianDy;
                }
            }

            // Quiet on lock error only (no FocusShift vote - too noisy for OK).
            // stillAccumUs counts only on-target time; brief gaps <= grace are paused.
            const bool onTarget =
                std::fabs(lastDy) <= holdDyBand &&
                std::fabs(lastVz) <= rateBand;
            if (onTarget) {
                stillBreakSinceUs = 0;
                if (stillSampleUs == 0) {
                    stillSampleUs = nowUs;
                    if (stillAccumUs == 0) {
                        stillThrSum = 0.0;
                        stillThrSamples = 0;
                    }
                } else if (nowUs > stillSampleUs) {
                    stillAccumUs += nowUs - stillSampleUs;
                    stillSampleUs = nowUs;
                }
                stillThrSum += static_cast<double>(throttle);
                stillThrSamples += 1;
            } else if (stillAccumUs != 0 || stillSampleUs != 0) {
                stillSampleUs = 0;
                if (stillBreakSinceUs == 0) {
                    stillBreakSinceUs = nowUs;
                }
                if (nowUs - stillBreakSinceUs >= kStillBreakGraceUs) {
                    stillAccumUs = 0;
                    stillBreakSinceUs = 0;
                    stillThrSum = 0.0;
                    stillThrSamples = 0;
                }
            }

            const double stillSec = stillAccumUs / 1.0e6;

            if (lastRegUs == 0 || nowUs - lastRegUs >= regIntervalUs) {
                const float stepDt = lastRegUs == 0
                    ? static_cast<float>(regIntervalUs) * 1.0e-6f
                    : static_cast<float>(nowUs - lastRegUs) * 1.0e-6f;
                lastRegUs = nowUs;

                const int holdFloor = static_cast<int>(stopClimb);
                const int holdCeil = static_cast<int>(liftoff);

                // I on height only near lock - large dy must not bleed hover to floor.
                if (std::fabs(lastDy) <= kHoverIntegrateDyPx &&
                    std::fabs(lastVz) <= rateBand * 2.0f) {
                    hoverTrim = AltitudePHold::integrateHover(
                        hoverTrim,
                        lastDy,
                        stepDt > 0.0f ? stepDt : dtReg,
                        holdFloor,
                        holdCeil,
                        kHoverKi);
                    hoverBase = hoverTrim;
                }

                const AltitudePHold::PStep step = AltitudePHold::stepCascade(
                    lastDy,
                    lastVz,
                    hoverTrim,
                    throttle,
                    holdFloor,
                    holdCeil,
                    vision,
                    slewMax,
                    true);
                if (!step.skipped && step.delta != 0) {
                    throttle = static_cast<uint16_t>(std::clamp(
                        static_cast<int>(throttle) + step.delta,
                        0,
                        static_cast<int>(kAxisMax)));
                }
                const char* action = step.frozen ? "FREEZE"
                    : (step.delta > 0 ? "ADD"
                        : (step.delta < 0 ? "CUT" : "HOLD"));
                std::printf(
                    "hover_trim: cas thr=%u hover=%u dy=%+.2f vz=%+.2f "
                    "vzCmd=%+.2f rateErr=%+.2f -> %+d %s on=%u still=%.2fs\n",
                    throttle,
                    hoverTrim,
                    lastDy,
                    lastVz,
                    step.vzCmd,
                    step.rateErr,
                    step.delta,
                    action,
                    onTarget ? 1u : 0u,
                    stillSec);
                std::fflush(stdout);
            }

            if (stillAccumUs >= stillNeedUs && stillThrSamples > 0) {
                const uint16_t measuredHover = static_cast<uint16_t>(
                    std::lround(stillThrSum / stillThrSamples));
                result.ok = true;
                result.hoverThrottle = measuredHover;
                hoverBase = measuredHover;
                hoverTrim = measuredHover;
                vision.present = true;
                phase = Phase::Done;
                std::printf(
                    "hover_trim: OK hover=%u Kp=%.2f Kv=%.1f vzMax=%.1f "
                    "still %.1fs at |dy|<=%.2f |vz|<=%.2f\n",
                    measuredHover,
                    vision.holdKpAlt,
                    vision.holdKv,
                    vision.holdVzMax,
                    stillNeedUs / 1.0e6,
                    holdDyBand,
                    rateBand);
                std::fflush(stdout);
                return false;
            }
        }

        writeStick(throttle);

        if (lastLogUs == 0 || nowUs - lastLogUs >= logEveryUs) {
            lastLogUs = nowUs;
            const char* name = phase == Phase::Hold
                ? "hold"
                : ApexClimb::phaseName(takeoff.phase);
            std::printf(
                "hover_trim: phase=%s thr=%u hover=%u dy=%+.2f vz=%+.2f still=%.1fs\n",
                name,
                throttle,
                hoverTrim,
                lastDy,
                lastVz,
                stillAccumUs / 1.0e6);
            std::fflush(stdout);
        }
        return phase != Phase::Done;
    };

    context.runtime.runLoop(tick, &context.session, "hover_trim", context.sessionId);

    if (context.runtime.stopped() && result.message.empty()) {
        result.message = "interrupted";
    }

    ControllerStick baseline = stick;
    baseline.throttle = static_cast<uint16_t>(
        std::lround(result.ok ? result.hoverThrottle : hoverBase));
    baseline.yaw = kAxisCenter;
    baseline.pitch = kAxisCenter;
    baseline.roll = kAxisCenter;
    context.session.setBaseline(baseline);
    context.session.close(false);

    std::ofstream meta(context.session.directory() + "/experiment.json", std::ios::binary);
    if (meta) {
        meta << "{\n  \"experiment\": \"hover_trim\",\n  \"status\": \""
             << (result.ok ? "OK" : "INCOMPLETE")
             << "\",\n  \"liftoffThrottle\": " << liftoff
             << ",\n  \"stopClimbThrottle\": " << stopClimb
             << ",\n  \"hoverThrottle\": " << (result.ok ? result.hoverThrottle : hoverBase)
             << ",\n  \"holdKpAlt\": " << vision.holdKpAlt
             << ",\n  \"holdKv\": " << vision.holdKv
             << ",\n  \"holdVzMax\": " << vision.holdVzMax
             << ",\n  \"stillSec\": " << (stillNeedUs / 1.0e6)
             << ",\n  \"holdBandPx\": " << holdDyBand
             << ",\n  \"heightLocked\": " << (altitudeLocked ? "true" : "false") << "\n}\n";
    }

    if (result.ok) {
        sticks.liftoffThrottle = liftoff;
        if (sticks.stopClimbThrottle <= 0.0) {
            sticks.stopClimbThrottle = stopClimb;
        }
        sticks.hoverThrottle = result.hoverThrottle;
        sticks.hoverMeasured = true;
        sticks.present = true;
        result.model = prior;
        result.model.setBaselineThrottle(result.hoverThrottle);
        result.model.setVisionMotion(vision);
        result.model.setSticks(sticks);
        saveDraftModel(result.model, context.session.directory(), context.activeModelPath, true);
        result.message = "hoverThrottle=" + std::to_string(result.hoverThrottle) +
            " Kp=" + std::to_string(vision.holdKpAlt) +
            " Kv=" + std::to_string(vision.holdKv);
    } else if (result.message.empty()) {
        result.message = "did not stay still on lock";
        StickCalibration cleared = prior.sticks();
        cleared.hoverMeasured = false;
        if (cleared.stopClimbThrottle <= 0.0) {
            cleared.stopClimbThrottle = stopClimb;
        }
        prior.setVisionMotion(vision);
        prior.setSticks(cleared);
        saveDraftModel(prior, context.session.directory(), context.activeModelPath, true);
    }

    std::printf(
        "hover_trim: done ok=%u liftoff=%u hover=%.0f Kp=%.2f Kv=%.1f lock=%u\n",
        result.ok ? 1u : 0u,
        liftoff,
        result.ok ? result.hoverThrottle : static_cast<double>(hoverBase),
        vision.holdKpAlt,
        vision.holdKv,
        altitudeLocked ? 1u : 0u);
    std::fflush(stdout);
    return result;
}
