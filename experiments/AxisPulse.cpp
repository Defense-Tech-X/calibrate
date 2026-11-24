#include "experiments/RollPulse.hpp"
#include "experiments/PitchPulse.hpp"

#include "Experiment.hpp"
#include "Runtime.hpp"
#include "hold/AltitudeHold.hpp"
#include "hold/LateralHold.hpp"
#include "motion/PointShift.hpp"
#include "takeoff/ApexClimb.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

namespace {

struct PulseSpec {
    const char* experimentName = nullptr;
    int axis = 3; // 0 throttle, 1 yaw, 2 pitch, 3 roll
    std::vector<int> amplitudes = { 100, 200, 400 };
    std::vector<uint64_t> durationsUs = { 150000, 250000, 400000 };
};

ExperimentResult runAxisPulse(ExperimentContext& context, const PulseSpec& spec) {
    ExperimentResult result;
    ArxModel prior;
    ControllerStick modelHover;
    if (!loadBaselineModel(context.activeModelPath, prior, modelHover) || !prior.hasHoverTrim()) {
        result.message = "measured hover trim required; run hover_trim after throttle_step";
        std::printf("%s: %s\n", spec.experimentName, result.message.c_str());
        std::fflush(stdout);
        return result;
    }
    if (prior.sticks().stopClimbThrottle <= 0.0) {
        result.message = "stopClimbThrottle required; run throttle_step first";
        std::printf("%s: %s\n", spec.experimentName, result.message.c_str());
        std::fflush(stdout);
        return result;
    }
    if (spec.axis == 2 && !prior.hasRollHold()) {
        result.message = "measured roll hold required; run roll_pulse before pitch_pulse";
        std::printf("%s: %s\n", spec.experimentName, result.message.c_str());
        std::fflush(stdout);
        return result;
    }

    if (!runResetPhase(context)) {
        result.message = "interrupted during reset";
        std::printf("%s: %s\n", spec.experimentName, result.message.c_str());
        std::fflush(stdout);
        return result;
    }

    VisionMotionParams vision = prior.visionMotion();
    if (vision.climbConfirmFrames < 3) {
        vision.climbConfirmFrames = 3;
    }
    const uint16_t liftoffThrottle = static_cast<uint16_t>(
        prior.sticks().liftoffThrottle > 0.0 ? prior.sticks().liftoffThrottle : prior.baseline()[0]);
    const uint16_t stopClimbThrottle = static_cast<uint16_t>(prior.sticks().stopClimbThrottle);
    const uint16_t measuredHover = static_cast<uint16_t>(prior.sticks().hoverThrottle);
    uint16_t hoverThrottle = measuredHover;

    ControllerStick hover = modelHover;
    hover.throttle = hoverThrottle;
    hover.yaw = kAxisCenter;
    hover.pitch = kAxisCenter;
    hover.roll = kAxisCenter;
    hover.flightMode = kFlightModeAngle;

    context.session.open(context.sessionId, context.sessionsRoot, spec.experimentName);

    size_t ampIndex = 0;
    size_t durIndex = 0;
    int sign = 1;
    // Pulse pipeline:
    //   shared ApexClimb (same as hover_trim) -> SoftCeil hold -> roll/pitch pulses
    enum class Phase { Takeoff, Lock, Stabilize, Hold, Pulse, Recover };
    Phase phase = Phase::Takeoff;
    uint64_t phaseStartUs = 0;
    uint64_t lastLogUs = 0;
    uint64_t lastTrimUs = 0;
    uint64_t altitudeLockUs = 0;
    const uint64_t stabilizeUs = 2500000;
    const uint64_t holdUs = 800000;
    const uint64_t recoverUs = 1000000;
    const uint64_t trimIntervalUs = 250000;
    const uint64_t holdRegIntervalUs = 120000;
    const uint64_t motionSampleUs = 150000;
    const uint64_t logEveryUs = 500000;
    const int holdFloorCap = AltitudeHold::floorThr(measuredHover);
    const int holdCeilCap = static_cast<int>(liftoffThrottle);
    ApexClimb::Config takeoffCfg;
    takeoffCfg.experimentName = spec.experimentName;
    takeoffCfg.liftoff = liftoffThrottle;
    takeoffCfg.stopClimb = stopClimbThrottle;
    takeoffCfg.vision = vision;
    ApexClimb::State takeoff;
    int holdFloorThr = holdFloorCap;
    int holdCeilThr = holdCeilCap;
    // pitch_pulse: SoftCeil + LAT (frozen during pulse). roll_pulse: SoftCeil only (LAT fights roll excite).
    const bool holdLateral = (spec.axis == 2);
    const bool measureRollHold = (spec.axis == 3);
    const AltitudeHold::AboveLockPolicy aboveLockPolicy =
        AltitudeHold::AboveLockPolicy::SoftCeil;
    const float holdRate = vision.hoverHoldRatePxPerSec < 1.0f ? 1.2f : vision.hoverHoldRatePxPerSec;
    const int holdSlewMax = AltitudeHold::kSlewMax;
    const int rollTrimStep = holdLateral
        ? std::max(1, static_cast<int>(std::lround(prior.sticks().rollHoldStep)))
        : 0;
    const int rollTrimMax = holdLateral
        ? std::max(rollTrimStep, static_cast<int>(std::lround(prior.sticks().rollHoldMax)))
        : 0;
    int holdRoll = 0;
    float latBias = 0.0f;
    uint64_t lastRollTrimUs = 0;
    bool haveLateralSample = false;
    float lastFocusDx = 0.0f;
    float lastLatRate = 0.0f;
    float prevFocusDx = 0.0f;
    uint64_t prevLateralSampleUs = 0;
    // roll_pulse measures smallest stick amp that produces clear lateral rate for pitch hold.
    int measuredRollStep = 0;
    int measuredRollMax = 0;
    float pulsePeakAbsLatRate = 0.0f;
    bool haveRollRespSample = false;
    float prevRollRespDx = 0.0f;
    uint64_t prevRollRespUs = 0;
    const char* lateralLabel = "LAT_OFF";
    bool phaseArmed = false;
    bool sessionBaselineSet = false;
    bool altitudeRefLocked = false;
    bool haveAltitudeSample = false;
    bool abortDrift = false;
    bool cloudPresent = false;
    float lastFocusDy = 0.0f;
    float lastFocusRate = 0.0f;
    float prevFocusDy = 0.0f;
    uint64_t prevAltitudeSampleUs = 0;
    uint32_t trimCount = 0;
    const char* verticalLabel = "UNKNOWN";
    std::vector<TrackedPoint> altitudeRef;
    std::vector<TrackedPoint> lastGoodPoints;
    const size_t totalCombos = spec.amplitudes.size() * spec.durationsUs.size() * 2;

    if (holdLateral) {
        std::printf(
            "%s: step4 pitch - hold alt SoftCeil + LAT P2 (freeze roll during pulse; step=%d max=%d); measure pitch stick\n"
            "%s: liftoff=%u hover=%u session=%s\n",
            spec.experimentName,
            rollTrimStep,
            rollTrimMax,
            spec.experimentName,
            liftoffThrottle,
            hoverThrottle,
            context.session.directory().c_str());
    } else if (measureRollHold) {
        std::printf(
            "%s: step3 roll - ApexClimb then SoftCeil (Kp=%.2f Kv=%.1f vzMax=%.1f); "
            "measure rollHoldStep/Max (no LAT - same axis as excite)\n"
            "%s: liftoff=%u stopClimb=%u hover=%u session=%s\n",
            spec.experimentName,
            vision.holdKpAlt > 0.0f ? vision.holdKpAlt : AltitudeHold::kKpAlt,
            vision.holdKv > 0.0f ? vision.holdKv : AltitudeHold::kKv,
            vision.holdVzMax > 0.0f ? vision.holdVzMax : AltitudeHold::kVzMax,
            spec.experimentName,
            liftoffThrottle,
            stopClimbThrottle,
            hoverThrottle,
            context.session.directory().c_str());
    } else {
        std::printf(
            "%s: liftoff=%u hover=%u session=%s\n",
            spec.experimentName,
            liftoffThrottle,
            hoverThrottle,
            context.session.directory().c_str());
    }
    std::fflush(stdout);

    auto rollCommand = [&]() -> uint16_t {
        if (!holdLateral) {
            return kAxisCenter;
        }
        return static_cast<uint16_t>(std::clamp(
            static_cast<int>(kAxisCenter) + holdRoll,
            0,
            static_cast<int>(kAxisMax)));
    };

    auto writeCentered = [&](uint16_t throttle) {
        ControllerStick command = hover;
        command.throttle = throttle;
        command.yaw = kAxisCenter;
        command.pitch = kAxisCenter;
        command.roll = rollCommand();
        context.runtime.writeStick(command);
        return command;
    };

    auto syncHoverBaseline = [&]() {
        hover.throttle = hoverThrottle;
        hover.roll = rollCommand();
        if (sessionBaselineSet) {
            context.session.setBaseline(hover);
        }
    };

    auto rememberCloud = [&](const PointCloud* cloud) {
        cloudPresent = cloud != nullptr && cloud->points.size() >= kMinMatchedPoints;
        if (cloudPresent) {
            lastGoodPoints = cloud->points;
        }
    };

    auto recenterHoldBand = [&]() {
        // Floor down to stopClimb so apex can keep coast thr (no raise-to-hover-400).
        holdFloorThr = std::min(holdFloorCap, static_cast<int>(stopClimbThrottle));
        holdCeilThr = holdCeilCap;
        if (holdCeilThr < static_cast<int>(hoverThrottle)) {
            holdCeilThr = static_cast<int>(hoverThrottle);
        }
        if (static_cast<int>(hoverThrottle) < holdFloorThr) {
            const uint16_t raised = static_cast<uint16_t>(holdFloorThr);
            std::printf(
                "%s: hold raise thr %u -> %u (holdFloor=%d)\n",
                spec.experimentName,
                hoverThrottle,
                raised,
                holdFloorThr);
            std::fflush(stdout);
            hoverThrottle = raised;
            syncHoverBaseline();
        }
        haveLateralSample = false;
        lastFocusDx = 0.0f;
        lastLatRate = 0.0f;
        prevFocusDx = 0.0f;
        prevLateralSampleUs = 0;
        if (holdLateral) {
            lateralLabel = "LAT_ARMED";
        }
        std::printf(
            "%s: hold limits thr=%u floor=%d ceil=%d (hover=%u)%s\n",
            spec.experimentName,
            hoverThrottle,
            holdFloorThr,
            holdCeilThr,
            measuredHover,
            holdLateral ? " SoftCeil + lateral roll hold (P2, freeze during pulse)"
                        : " SoftCeil (hover_trim cascade)");
        std::fflush(stdout);
    };

    auto lockAltitudePoints = [&](uint64_t nowUs, const std::vector<TrackedPoint>& points, const char* why) -> bool {
        if (points.size() < kMinMatchedPoints) {
            return false;
        }
        altitudeRef = points;
        altitudeRefLocked = true;
        altitudeLockUs = nowUs;
        haveAltitudeSample = false;
        prevFocusDy = 0.0f;
        prevAltitudeSampleUs = 0;
        lastFocusDy = 0.0f;
        lastFocusRate = 0.0f;
        verticalLabel = "PICTURE_LOCKED";
        recenterHoldBand();
        std::printf(
            "%s: PICTURE LOCKED (%s) points=%u thr=%u - climb has stopped; hold THIS frame\n",
            spec.experimentName,
            why,
            static_cast<unsigned>(altitudeRef.size()),
            hoverThrottle);
        std::fflush(stdout);
        return true;
    };

    auto lockAltitudeReference = [&](uint64_t nowUs, const PointCloud* cloud, const char* why) -> bool {
        if (cloud != nullptr && cloud->points.size() >= kMinMatchedPoints) {
            return lockAltitudePoints(nowUs, cloud->points, why);
        }
        if (!lastGoodPoints.empty()) {
            return lockAltitudePoints(nowUs, lastGoodPoints, why);
        }
        return false;
    };

    auto applyThrDelta = [&](int delta, uint64_t nowUs, const char* why, int floorThr, int ceilThr) -> bool {
        if (delta == 0) {
            return false;
        }
        if (lastTrimUs != 0 && nowUs - lastTrimUs < trimIntervalUs) {
            return false;
        }
        const int current = static_cast<int>(hoverThrottle);
        int next = std::clamp(current + delta, floorThr, ceilThr);
        // Never reverse commanded direction (old bug: cut while thr < measuredHover-band
        // clamped UP to floor and sent the craft to the sky).
        if (delta < 0 && next > current) {
            std::printf(
                "%s: thr REFUSED reverse-clamp at %u (want %+d but floor=%d would RAISE) VERT=%s dy=%.2f rate=%.2f [%s]\n",
                spec.experimentName,
                hoverThrottle,
                delta,
                floorThr,
                verticalLabel,
                lastFocusDy,
                lastFocusRate,
                why);
            std::fflush(stdout);
            lastTrimUs = nowUs;
            return false;
        }
        if (delta > 0 && next < current) {
            std::printf(
                "%s: thr REFUSED reverse-clamp at %u (want %+d but ceil=%d would LOWER) VERT=%s dy=%.2f rate=%.2f [%s]\n",
                spec.experimentName,
                hoverThrottle,
                delta,
                ceilThr,
                verticalLabel,
                lastFocusDy,
                lastFocusRate,
                why);
            std::fflush(stdout);
            lastTrimUs = nowUs;
            return false;
        }
        if (next == current) {
            std::printf(
                "%s: thr CLAMPED at %u floor=%d ceil=%d (want %+d) VERT=%s dy=%.2f rate=%.2f [%s]\n",
                spec.experimentName,
                hoverThrottle,
                floorThr,
                ceilThr,
                delta,
                verticalLabel,
                lastFocusDy,
                lastFocusRate,
                why);
            std::fflush(stdout);
            lastTrimUs = nowUs;
            return false;
        }
        const uint16_t previous = hoverThrottle;
        hoverThrottle = static_cast<uint16_t>(next);
        syncHoverBaseline();
        lastTrimUs = nowUs;
        ++trimCount;
        std::printf(
            "%s: thr %u -> %u (cmd %+d actual %+d) VERT=%s dy=%.2f rate=%.2f [%s]\n",
            spec.experimentName,
            previous,
            hoverThrottle,
            delta,
            next - current,
            verticalLabel,
            lastFocusDy,
            lastFocusRate,
            why);
        std::fflush(stdout);
        return true;
    };

    // Lateral hold (pitch_pulse / P2): see hold/LateralHold.hpp.
    const float latPosDeadband = LateralHold::posDeadband(vision.hoverHoldDyPx);
    const float latKr = LateralHold::rateGain(rollTrimStep, holdRate);
    const int latSlewMax = LateralHold::slewMax(rollTrimStep);

    auto maybeHoldLateral = [&](uint64_t nowUs, const FocusShift& focus, const char* where) {
        if (!holdLateral || !altitudeRefLocked) {
            return;
        }
        const float dx = focus.focusMedianDx;
        if (!haveLateralSample) {
            prevFocusDx = dx;
            prevLateralSampleUs = nowUs;
            haveLateralSample = true;
            lastFocusDx = dx;
            lastLatRate = 0.0f;
            lateralLabel = "LAT_ARMED";
            return;
        }
        const float dtSec = (nowUs - prevLateralSampleUs) / 1.0e6f;
        if (dtSec * 1.0e6f < static_cast<float>(motionSampleUs)) {
            lastFocusDx = dx;
            return;
        }
        const float latRate = (dx - prevFocusDx) / std::max(dtSec, 1.0e-3f);
        prevFocusDx = dx;
        prevLateralSampleUs = nowUs;
        lastFocusDx = dx;
        lastLatRate = latRate;

        if (nowUs - altitudeLockUs < 200000) {
            lateralLabel = "LAT_STILL";
            return;
        }
        // Pitch excitation couples into dx/latRate; keep last holdRoll through the pulse.
        if (std::strcmp(where, "pulse") == 0) {
            lateralLabel = "LAT_FROZEN";
            return;
        }

        latBias = LateralHold::updateBias(
            latBias, dx, dtSec, latPosDeadband, rollTrimStep, rollTrimMax);

        const uint64_t latTrimIntervalUs =
            LateralHold::trimIntervalUs(trimIntervalUs, dx, latPosDeadband);
        if (lastRollTrimUs != 0 && nowUs - lastRollTrimUs < latTrimIntervalUs) {
            return;
        }

        const float latRateCmd = LateralHold::clampRate(latRate, holdRate);
        const bool rateQuiet = std::fabs(latRateCmd) < holdRate;
        const bool posQuiet = std::fabs(dx) < latPosDeadband;
        const char* craftDir = "STILL";
        if (!rateQuiet) {
            craftDir = latRateCmd > 0.0f ? "LEFT" : "RIGHT";
        } else if (!posQuiet) {
            craftDir = dx > 0.0f ? "LEFT" : "RIGHT";
        }
        lateralLabel = rateQuiet && posQuiet
            ? "LAT_STILL"
            : (latRateCmd > 0.0f || (rateQuiet && dx > 0.0f) ? "LAT_LEFT" : "LAT_RIGHT");

        const float kx = LateralHold::effectivePosGain(rollTrimStep, dx, latPosDeadband);
        int target = static_cast<int>(std::lround(latBias));
        if (!(rateQuiet && posQuiet)) {
            const float rateTerm = rateQuiet ? 0.0f : latRateCmd;
            const float posTerm = posQuiet ? 0.0f : dx;
            target = static_cast<int>(std::lround(latKr * rateTerm + kx * posTerm + latBias));
        }
        target = std::clamp(target, -rollTrimMax, rollTrimMax);

        int delta = target - holdRoll;
        if (delta > latSlewMax) {
            delta = latSlewMax;
        } else if (delta < -latSlewMax) {
            delta = -latSlewMax;
        }
        if (delta == 0) {
            if (!rateQuiet || !posQuiet || holdRoll != 0 || std::fabs(latBias) > 0.5f) {
                std::printf(
                    "%s: LAT_HOLD craft=%s latRate=%+.2f dx=%.2f roll=%+d target=%+d "
                    "bias=%+.1f (P2 hold; oppose %s with roll %s) [%s]\n",
                    spec.experimentName,
                    craftDir,
                    latRate,
                    dx,
                    holdRoll,
                    target,
                    latBias,
                    craftDir,
                    target > 0 ? "+" : (target < 0 ? "-" : "0"),
                    where);
                std::fflush(stdout);
            }
            return;
        }

        const int previous = holdRoll;
        holdRoll = std::clamp(holdRoll + delta, -rollTrimMax, rollTrimMax);
        syncHoverBaseline();
        lastRollTrimUs = nowUs;
        std::printf(
            "%s: LAT_HOLD craft=%s latRate=%+.2f dx=%.2f roll=%+d -> %+d "
            "target=%+d slew=%+d bias=%+.1f (P2 Kr/8+bias clamp3x; LEFT->roll+) [%s]\n",
            spec.experimentName,
            craftDir,
            latRate,
            dx,
            previous,
            holdRoll,
            target,
            holdRoll - previous,
            latBias,
            where);
        std::fflush(stdout);
    };

    // Cascaded altitude hold = hover_trim law (AltitudePHold via AltitudeHold).
    auto maybeHoldPicture = [&](uint64_t nowUs, const PointCloud* cloud, const char* where) {
        if (!altitudeRefLocked) {
            return;
        }
        if (cloud == nullptr || cloud->points.size() < kMinMatchedPoints) {
            return;
        }
        const FocusShift focus = measureFocusShift(altitudeRef, cloud->points, vision);
        if (!focus.valid) {
            return;
        }
        maybeHoldLateral(nowUs, focus, where);
        if (focus.lateralDominant && !holdLateral) {
            return;
        }

        const float dy = focus.focusMedianDy;
        if (!haveAltitudeSample) {
            prevFocusDy = dy;
            prevAltitudeSampleUs = nowUs;
            haveAltitudeSample = true;
            lastFocusDy = dy;
            lastFocusRate = 0.0f;
            verticalLabel = "HOLD_ARMED";
            return;
        }
        const float dtSec = (nowUs - prevAltitudeSampleUs) / 1.0e6f;
        if (dtSec * 1.0e6f < static_cast<float>(motionSampleUs)) {
            lastFocusDy = dy;
            return;
        }
        const float instRate = (dy - prevFocusDy) / dtSec;
        prevFocusDy = dy;
        prevAltitudeSampleUs = nowUs;
        lastFocusDy = dy;
        lastFocusRate = instRate;

        // Keep the apex picture fixed (no still-relock). Below lock always run
        // cascade so SoftFloor/ADD can recover; quiet FREEZE lives inside stepCascade.
        if (nowUs - altitudeLockUs < 200000) {
            verticalLabel = "HOLD_STILL";
            std::printf(
                "%s: MOTION=STILL rate=%+.2f dy=%.2f thr=%u [%s] (grace after lock)\n",
                spec.experimentName,
                instRate,
                dy,
                hoverThrottle,
                where);
            std::fflush(stdout);
            return;
        }

        if (lastTrimUs != 0 && nowUs - lastTrimUs < holdRegIntervalUs) {
            verticalLabel = instRate < 0.0f ? "HOLD_DOWN" : "HOLD_UP";
            return;
        }

        const AltitudeHold::CascadeStep step = AltitudeHold::stepCascade(
            dy,
            instRate,
            measuredHover,
            hoverThrottle,
            holdFloorThr,
            holdCeilThr,
            vision,
            aboveLockPolicy,
            AltitudeHold::BelowLockPolicy::SoftFloor,
            holdSlewMax);
        if (step.skipped) {
            return;
        }

        const int delta = step.delta;
        const char* holdTag = "SoftCeil";
        const char* softTag = step.frozen ? " FREEZE" : "";

        const char* craftVert = "STILL";
        if (instRate > holdRate) {
            craftVert = "UP";
        } else if (instRate < -holdRate) {
            craftVert = "DOWN";
        }
        const char* action = step.frozen ? "FREEZE"
            : (delta < 0 ? "CUT" : (delta > 0 ? "ADD" : "HOLD"));

        if (delta > 0 || (delta == 0 && step.rateErr < 0.0f)) {
            verticalLabel = "HOLD_DOWN";
        } else if (delta < 0 || step.rateErr > 0.0f) {
            verticalLabel = "HOLD_UP";
        } else {
            verticalLabel = "HOLD_STILL";
        }

        std::printf(
            "%s: MOTION=ALT craft=%s rate=%+.2f vzCmd=%+.2f dy=%.2f thr=%u -> cascade %+d "
            "action=%s (%s target=%d hover=%u floor=%d ceil=%d%s) [%s]\n",
            spec.experimentName,
            craftVert,
            instRate,
            step.vzCmd,
            dy,
            hoverThrottle,
            delta,
            action,
            holdTag,
            step.limitedTarget,
            measuredHover,
            holdFloorThr,
            holdCeilThr,
            softTag,
            where);
        std::fflush(stdout);
        if (delta == 0) {
            return;
        }
        lastTrimUs = 0;
        applyThrDelta(delta, nowUs, where, holdFloorThr, holdCeilThr);
    };

    auto tick = [&](uint64_t nowUs,
                    const ControllerStick& current,
                    const PointCloud* cloud,
                    const PointShift*,
                    const ObservationSample* /*observation*/,
                    bool referenceReady) -> bool {
        if (context.runtime.stopped()) {
            result.message = "interrupted";
            return false;
        }
        if (abortDrift) {
            result.message = "altitude drift: still moving after picture lock";
            return false;
        }
        if (!phaseArmed) {
            phaseArmed = true;
            ApexClimb::begin(takeoff, takeoffCfg, nowUs);
            phaseStartUs = nowUs;
        }
        rememberCloud(cloud);

        ControllerStick command = hover;
        const char* phaseName = "takeoff";
        int pulseAmp = 0;

        auto enterLock = [&](uint64_t now, const char* why) {
            phase = Phase::Lock;
            phaseStartUs = now;
            if (!sessionBaselineSet) {
                hover.throttle = hoverThrottle;
                context.session.setBaseline(hover);
                sessionBaselineSet = true;
            }
            std::printf(
                "%s: %s -> request observation lock; hold picture thr=%u\n",
                spec.experimentName,
                why,
                hoverThrottle);
            std::fflush(stdout);
        };

        if (phase == Phase::Takeoff) {
            phaseName = ApexClimb::phaseName(takeoff.phase);
            const ApexClimb::StepResult takeoffStep =
                ApexClimb::step(takeoff, takeoffCfg, nowUs, cloud);
            command = writeCentered(takeoff.throttle);
            verticalLabel = phaseName;
            if (!takeoffStep.ok) {
                result.message = takeoffStep.failMessage != nullptr
                    ? takeoffStep.failMessage
                    : "apex takeoff failed";
                return false;
            }
            if (takeoffStep.justLocked || takeoff.phase == ApexClimb::Phase::Locked) {
                // Keep coast dump thr (stopClimb). Jumping to measuredHover pumps climb.
                hoverThrottle = takeoff.throttle;
                syncHoverBaseline();
                if (!lockAltitudePoints(
                        nowUs, takeoff.altitudeRef, takeoff.lockWhy.c_str())) {
                    result.message = "apex lock without points";
                    return false;
                }
                enterLock(nowUs, "apex locked");
            }
        } else if (phase == Phase::Lock) {
            phaseName = "lock";
            maybeHoldPicture(nowUs, cloud, "lock");
            command = writeCentered(hoverThrottle);
            context.runtime.observation().requestLock(context.sessionId, context.session.keyframePath());
            if (abortDrift) {
                return false;
            }
            if (referenceReady) {
                phase = Phase::Stabilize;
                phaseStartUs = nowUs;
                std::printf(
                    "%s: observation ready, stabilize thr=%u for %.1fs (HOLD picture)\n",
                    spec.experimentName,
                    hoverThrottle,
                    stabilizeUs / 1.0e6);
                std::fflush(stdout);
            }
        } else if (phase == Phase::Stabilize) {
            phaseName = "stabilize";
            maybeHoldPicture(nowUs, cloud, "stabilize");
            command = writeCentered(hoverThrottle);
            if (abortDrift) {
                return false;
            }
            if (nowUs - phaseStartUs >= stabilizeUs) {
                phase = Phase::Hold;
                phaseStartUs = nowUs;
                std::printf(
                    "%s: stabilize done thr=%u trims=%u VERT=%s, starting pulses\n",
                    spec.experimentName,
                    hoverThrottle,
                    trimCount,
                    verticalLabel);
                std::fflush(stdout);
            }
        } else if (phase == Phase::Hold) {
            phaseName = "hold";
            maybeHoldPicture(nowUs, cloud, "hold");
            command = writeCentered(hoverThrottle);
            if (abortDrift) {
                return false;
            }
            if (nowUs - phaseStartUs >= holdUs) {
                phase = Phase::Pulse;
                phaseStartUs = nowUs;
                pulsePeakAbsLatRate = 0.0f;
                haveRollRespSample = false;
                prevRollRespUs = 0;
                std::printf(
                    "%s: pulse amp=%d sign=%d dur=%.2fs (%zu/%zu) thr=%u VERT=%s rollTrim=%d\n",
                    spec.experimentName,
                    spec.amplitudes[ampIndex],
                    sign,
                    spec.durationsUs[durIndex] / 1.0e6,
                    ampIndex * spec.durationsUs.size() * 2 + durIndex * 2 + (sign < 0 ? 1 : 0) + 1,
                    totalCombos,
                    hoverThrottle,
                    verticalLabel,
                    holdLateral ? holdRoll : 0);
                std::fflush(stdout);
            }
        } else if (phase == Phase::Pulse) {
            phaseName = "pulse";
            maybeHoldPicture(nowUs, cloud, "pulse");
            if (abortDrift) {
                return false;
            }
            pulseAmp = spec.amplitudes[ampIndex] * sign;
            const uint64_t duration = spec.durationsUs[durIndex];
            int deltas[4] = {};
            deltas[spec.axis] = pulseAmp;
            command.throttle = hoverThrottle;
            command.yaw = static_cast<uint16_t>(std::clamp(static_cast<int>(kAxisCenter) + deltas[1], 0, static_cast<int>(kAxisMax)));
            command.pitch = static_cast<uint16_t>(std::clamp(static_cast<int>(kAxisCenter) + deltas[2], 0, static_cast<int>(kAxisMax)));
            // roll_pulse: excite roll. pitch_pulse: keep LAT trim while pitching.
            if (holdLateral) {
                command.roll = rollCommand();
            } else {
                command.roll = static_cast<uint16_t>(std::clamp(
                    static_cast<int>(kAxisCenter) + deltas[3], 0, static_cast<int>(kAxisMax)));
            }
            if (measureRollHold && altitudeRefLocked && cloud != nullptr &&
                cloud->points.size() >= kMinMatchedPoints) {
                const FocusShift focus = measureFocusShift(altitudeRef, cloud->points, vision);
                if (focus.valid) {
                    const float dx = focus.focusMedianDx;
                    if (!haveRollRespSample) {
                        prevRollRespDx = dx;
                        prevRollRespUs = nowUs;
                        haveRollRespSample = true;
                    } else {
                        const float dtSec = (nowUs - prevRollRespUs) / 1.0e6f;
                        if (dtSec * 1.0e6f >= static_cast<float>(motionSampleUs)) {
                            const float latRate = (dx - prevRollRespDx) / std::max(dtSec, 1.0e-3f);
                            prevRollRespDx = dx;
                            prevRollRespUs = nowUs;
                            lastFocusDx = dx;
                            lastLatRate = latRate;
                            pulsePeakAbsLatRate = std::max(pulsePeakAbsLatRate, std::fabs(latRate));
                        }
                    }
                }
            }
            if (nowUs - phaseStartUs >= duration) {
                if (measureRollHold) {
                    const int amp = std::abs(pulseAmp);
                    const bool effective = pulsePeakAbsLatRate >= holdRate;
                    std::printf(
                        "%s: ROLL_RESP amp=%d peakAbsLatRate=%.2f need>=%.2f -> %s\n",
                        spec.experimentName,
                        amp,
                        pulsePeakAbsLatRate,
                        holdRate,
                        effective ? "EFFECTIVE" : "weak");
                    std::fflush(stdout);
                    if (effective) {
                        if (measuredRollStep == 0 || amp < measuredRollStep) {
                            measuredRollStep = amp;
                        }
                        if (amp > measuredRollMax) {
                            measuredRollMax = amp;
                        }
                    }
                }
                phase = Phase::Recover;
                phaseStartUs = nowUs;
            }
        } else {
            phaseName = "recover";
            maybeHoldPicture(nowUs, cloud, "recover");
            command = writeCentered(hoverThrottle);
            if (abortDrift) {
                return false;
            }
            if (nowUs - phaseStartUs >= recoverUs) {
                if (sign > 0) {
                    sign = -1;
                } else {
                    sign = 1;
                    ++durIndex;
                    if (durIndex >= spec.durationsUs.size()) {
                        durIndex = 0;
                        ++ampIndex;
                    }
                    if (ampIndex >= spec.amplitudes.size()) {
                        result.ok = true;
                        if (measureRollHold) {
                            std::printf(
                                "%s: all pulses done thr=%u; rollHoldStep=%d rollHoldMax=%d (from EFFECTIVE amps)\n",
                                spec.experimentName,
                                hoverThrottle,
                                measuredRollStep,
                                measuredRollMax);
                        } else {
                            std::printf(
                                "%s: all pulses done (final thr=%u, micro-trims=%u, VERT=%s)\n",
                                spec.experimentName,
                                hoverThrottle,
                                trimCount,
                                verticalLabel);
                        }
                        std::fflush(stdout);
                        return false;
                    }
                }
                phase = Phase::Hold;
                phaseStartUs = nowUs;
            }
        }

        if (phase != Phase::Takeoff && phase != Phase::Lock) {
            if (command.throttle != current.throttle || command.yaw != current.yaw || command.pitch != current.pitch ||
                command.roll != current.roll) {
                context.runtime.writeStick(command);
            }
        }

        if (lastLogUs == 0 || nowUs - lastLogUs >= logEveryUs) {
            lastLogUs = nowUs;
            std::printf(
                "%s: phase=%s VERT=%s LAT=%s thr=%u roll=%+d amp=%d dy=%.2f rate=%+.2f dx=%.2f latRate=%+.2f pic=%u\n",
                spec.experimentName,
                phaseName,
                verticalLabel,
                lateralLabel,
                command.throttle,
                holdLateral ? holdRoll : 0,
                pulseAmp,
                lastFocusDy,
                lastFocusRate,
                lastFocusDx,
                lastLatRate,
                altitudeRefLocked ? 1u : 0u);
            std::fflush(stdout);
        }
        return true;
    };

    context.runtime.runLoop(tick, &context.session, spec.experimentName, context.sessionId);
    if (!sessionBaselineSet) {
        hover.throttle = hoverThrottle;
        context.session.setBaseline(hover);
    }
    context.session.close(false);

    const std::vector<ArxRow> rows = context.session.collectFitRows();
    // Fit baseline uses the session operating point; sticks keep hover_trim measurement.
    double baseline[4] = {
        static_cast<double>(hoverThrottle),
        prior.baseline()[1],
        prior.baseline()[2],
        prior.baseline()[3] };
    result.model = fitArxModel(rows, baseline);
    StickCalibration sticks = prior.sticks();
    sticks.hoverThrottle = measuredHover;
    sticks.hoverMeasured = true;
    sticks.present = true;
    if (sticks.liftoffThrottle <= 0.0) {
        sticks.liftoffThrottle = liftoffThrottle;
    }
    if (measureRollHold) {
        if (measuredRollStep > 0) {
            sticks.rollHoldStep = measuredRollStep;
            sticks.rollHoldMax = measuredRollMax > 0 ? measuredRollMax : measuredRollStep;
            sticks.rollMeasured = true;
        } else if (result.ok && !spec.amplitudes.empty()) {
            // Pulses finished but vision never saw clear LAT - still publish schedule bounds.
            sticks.rollHoldStep = spec.amplitudes.front();
            sticks.rollHoldMax = spec.amplitudes.back();
            sticks.rollMeasured = true;
            std::printf(
                "%s: WARN no EFFECTIVE ROLL_RESP; saved schedule step=%g max=%g for pitch hold\n",
                spec.experimentName,
                sticks.rollHoldStep,
                sticks.rollHoldMax);
            std::fflush(stdout);
        }
    }
    result.model.setVisionMotion(prior.visionMotion());
    result.model.setSticks(sticks);
    result.model.setBaselineThrottle(measuredHover);
    result.hoverThrottle = measuredHover;

    std::ofstream meta(context.session.directory() + "/experiment.json", std::ios::binary);
    if (meta) {
        meta << "{\n  \"experiment\": \"" << spec.experimentName << "\",\n  \"status\": \""
             << (result.ok ? "OK" : "INCOMPLETE") << "\",\n  \"liftoffThrottle\": " << liftoffThrottle
             << ",\n  \"measuredHoverThrottle\": " << measuredHover
             << ",\n  \"sessionHoverThrottle\": " << hoverThrottle
             << ",\n  \"microTrimCount\": " << trimCount
             << ",\n  \"rollHoldStep\": " << sticks.rollHoldStep
             << ",\n  \"rollHoldMax\": " << sticks.rollHoldMax
             << ",\n  \"rollMeasured\": " << (sticks.rollMeasured ? "true" : "false")
             << ",\n  \"samples\": " << rows.size() << "\n}\n";
    }

    result.model.setAppliedToControl(false);
    const bool promote = result.model.status() == ArxStatus::Ok;
    // Session draft always; active gets the fitted OK model (sticks already on result.model).
    saveDraftModel(result.model, context.session.directory(), context.activeModelPath, promote);
    // Weak ARX: keep prior plant, refresh sticks/hover on active only (do not clobber a promoted OK).
    if (!promote) {
        StickCalibration keepSticks = prior.sticks();
        keepSticks.hoverThrottle = measuredHover;
        keepSticks.hoverMeasured = true;
        keepSticks.present = true;
        if (keepSticks.liftoffThrottle <= 0.0) {
            keepSticks.liftoffThrottle = liftoffThrottle;
        }
        if (measureRollHold && sticks.rollMeasured) {
            keepSticks.rollHoldStep = sticks.rollHoldStep;
            keepSticks.rollHoldMax = sticks.rollHoldMax;
            keepSticks.rollMeasured = true;
        }
        prior.setSticks(keepSticks);
        prior.setBaselineThrottle(measuredHover);
        prior.save(context.activeModelPath);
        std::printf("wrote active sticks (ARX not OK): %s\n", context.activeModelPath.c_str());
        std::fflush(stdout);
    }
    if (result.message.empty()) {
        result.message = result.ok ? "pulses complete" : "aborted";
    }
    if (trimCount > 0) {
        result.message += "; session micro-trim ended at " + std::to_string(hoverThrottle) +
            " (active hover kept " + std::to_string(measuredHover) + ")";
    }
    if (measureRollHold && sticks.rollMeasured) {
        result.message += "; rollHoldStep=" + std::to_string(static_cast<int>(sticks.rollHoldStep)) +
            " rollHoldMax=" + std::to_string(static_cast<int>(sticks.rollHoldMax));
    }
    if (result.model.status() == ArxStatus::Ok) {
        result.message += "; model OK (validate_forecast to apply)";
    }
    return result;
}

} // namespace

const char* RollPulseExperiment::name() const {
    return "roll_pulse";
}

ExperimentResult RollPulseExperiment::run(ExperimentContext& context) {
    PulseSpec spec;
    spec.experimentName = "roll_pulse";
    spec.axis = 3;
    return runAxisPulse(context, spec);
}

const char* PitchPulseExperiment::name() const {
    return "pitch_pulse";
}

ExperimentResult PitchPulseExperiment::run(ExperimentContext& context) {
    PulseSpec spec;
    spec.experimentName = "pitch_pulse";
    spec.axis = 2;
    return runAxisPulse(context, spec);
}
