#include "experiments/ThrottleStep.hpp"

#include "Experiment.hpp"
#include "Runtime.hpp"
#include "hold/AltitudeHold.hpp"
#include "identify/StickCalibration.hpp"
#include "identify/VisionMotionParams.hpp"
#include "motion/VisualVertical.hpp"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

void writeExperimentJson(
    const std::string& path,
    double liftoff,
    double stopClimb,
    int normalizationVersion,
    const char* status) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return;
    }
    out << "{\n  \"experiment\": \"throttle_step\",\n  \"status\": \"" << status << "\",\n"
        << "  \"liftoffThrottle\": " << liftoff << ",\n"
        << "  \"stopClimbThrottle\": " << stopClimb << ",\n"
        << "  \"normalizationVersion\": " << normalizationVersion << "\n}\n";
}

const char* phaseName(bool cutting) {
    return cutting ? "cut" : "raise";
}

} // namespace

const char* ThrottleStepExperiment::name() const {
    return "throttle_step";
}

ExperimentResult ThrottleStepExperiment::run(ExperimentContext& context) {
    ExperimentResult result;
    if (!runResetPhase(context)) {
        result.message = "interrupted during reset";
        return result;
    }

    SessionLog& session = context.session;
    session.open(context.sessionId, context.sessionsRoot, "throttle_step");

    VisionMotionParams vision;
    ArxModel prior;
    ControllerStick priorStick;
    const bool havePrior = loadBaselineModel(context.activeModelPath, prior, priorStick);
    if (havePrior) {
        vision = prior.visionMotion();
    }
    vision.present = true;

    enum class Phase { Raise, Cut };
    Phase phase = Phase::Raise;

    const uint16_t thrStep = 80;
    uint16_t throttle = 0;
    uint16_t liftoffThr = 0;
    uint16_t stopClimbThr = 0;
    uint64_t lastStepUs = 0;
    uint64_t lastLogUs = 0;
    uint32_t pointFrames = 0;
    const uint64_t settleUs = kStepUs;
    const uint64_t logEveryUs = 500000;

    VisualVerticalChannel channel;
    int passportVersion = -1;
    size_t passportPoints = 0;
    const std::string channelPath = session.directory() + "/visual_channel.jsonl";
    const std::string passportPath = session.directory() + "/visual_scale.json";
    std::ofstream channelLog(channelPath, std::ios::binary);
    auto writePassport = [&]() {
        std::ofstream passport(passportPath, std::ios::binary);
        if (passport) {
            passport << channel.passportJson();
        }
    };

    std::printf(
        "throttle_step: visual vertical channel; step %u / %.1fs\n"
        "throttle_step: valid climb hypothesis -> liftoff; dump; valid descent hypothesis -> stopClimb\n"
        "throttle_step: session=%s\n",
        thrStep,
        settleUs / 1.0e6,
        session.directory().c_str());
    std::fflush(stdout);

    auto tick = [&](uint64_t nowUs,
                    const ControllerStick& current,
                    const PointCloud* cloud,
                    const PointShift*,
                    const ObservationSample*,
                    bool) -> bool {
        if (context.runtime.stopped()) {
            result.message = "interrupted";
            return false;
        }
        if (lastStepUs == 0) {
            lastStepUs = nowUs;
        }

        ControllerStick command = current;
        command.yaw = kAxisCenter;
        command.pitch = kAxisCenter;
        command.roll = kAxisCenter;
        command.flightMode = kFlightModeAngle;

        bool closedPhase = false;
        const VisualChannelReport* report = nullptr;
        if (cloud != nullptr && !cloud->points.empty()) {
            VisualFrameInput input;
            input.flightId = context.sessionId;
            input.sequence = cloud->sequence;
            input.timestampUs = cloud->captureTimestampUs;
            input.throttle = throttle;
            input.points = &cloud->points;
            channel.observe(input);
            report = &channel.latest();
            ++pointFrames;
            if (channelLog) {
                channelLog << channel.reportJson() << "\n";
                channelLog.flush();
            }
            if (channel.passport().normalizationVersion != passportVersion ||
                channel.passport().pointIds.size() != passportPoints) {
                passportVersion = channel.passport().normalizationVersion;
                passportPoints = channel.passport().pointIds.size();
                writePassport();
            }

            const bool cleanSpeed = report->speedValid && !report->zeroMotion;
            if (phase == Phase::Raise && cleanSpeed && report->visualVerticalSpeedPxPerSecond > 0.0) {
                liftoffThr = throttle;
                throttle = static_cast<uint16_t>(AltitudeHold::settleDumpThr(liftoffThr));
                phase = Phase::Cut;
                lastStepUs = nowUs;
                closedPhase = true;
                std::printf(
                    "throttle_step: LIFTOFF thr=%u -> dump %u; height=%.2f speed=%.2f px/s\n",
                    liftoffThr,
                    throttle,
                    report->visualHeightPx,
                    report->visualVerticalSpeedPxPerSecond);
                std::fflush(stdout);
            } else if (phase == Phase::Cut && cleanSpeed && report->visualVerticalSpeedPxPerSecond < 0.0) {
                stopClimbThr = throttle;
                result.ok = true;
                result.hoverThrottle = stopClimbThr;
                closedPhase = true;
                std::printf(
                    "throttle_step: STOP_CLIMB thr=%u (liftoff was %u) height=%.2f speed=%.2f px/s\n",
                    stopClimbThr,
                    liftoffThr,
                    report->visualHeightPx,
                    report->visualVerticalSpeedPxPerSecond);
                std::fflush(stdout);
            }
        }

        bool stepped = false;
        if (!closedPhase && phase == Phase::Raise && result.message.empty()) {
            if (nowUs - lastStepUs >= settleUs) {
                const uint32_t next = static_cast<uint32_t>(throttle) + thrStep;
                if (next > kAxisMax) {
                    result.message = "max throttle without liftoff";
                    return false;
                }
                throttle = static_cast<uint16_t>(next);
                lastStepUs = nowUs;
                stepped = true;
            }
        } else if (!closedPhase && phase == Phase::Cut && result.message.empty()) {
            if (nowUs - lastStepUs >= settleUs) {
                if (throttle <= thrStep) {
                    result.message = "thr=0 without descent";
                    return false;
                }
                throttle = static_cast<uint16_t>(throttle - thrStep);
                lastStepUs = nowUs;
                stepped = true;
            }
        }

        if (result.ok) {
            return false;
        }

        command.throttle = throttle;
        if (command.throttle != current.throttle || command.roll != current.roll ||
            command.pitch != current.pitch || command.yaw != current.yaw) {
            context.runtime.writeStick(command);
        }

        if (lastLogUs == 0 || nowUs - lastLogUs >= logEveryUs || stepped || closedPhase) {
            lastLogUs = nowUs;
            if (pointFrames > 0) {
                const VisualChannelReport& shown = channel.latest();
                std::printf("throttle_step: stage=%s thr=%u ", phaseName(phase == Phase::Cut), throttle);
                if (shown.heightValid) {
                    std::printf("height=%.2f ", shown.visualHeightPx);
                } else {
                    std::printf("height=%s ", visualInvalidReasonName(shown.heightReason));
                }
                if (shown.speedValid) {
                    std::printf("speed=%.2f ", shown.visualVerticalSpeedPxPerSecond);
                } else {
                    std::printf("speed=%s ", visualInvalidReasonName(shown.speedReason));
                }
                std::printf(
                    "quality=%.2f points=%u frames=%u%s%s%s\n",
                    shown.quality,
                    shown.heightPointsUsed,
                    shown.speedFramesUsed,
                    shown.zeroMotion ? " zero" : "",
                    shown.rotationOrExtraneousMotion ? " rotation" : "",
                    stepped ? " [step]" : "");
            } else {
                std::printf(
                    "throttle_step: stage=%s thr=%u waiting for points%s\n",
                    phaseName(phase == Phase::Cut),
                    throttle,
                    stepped ? " [step]" : "");
            }
            std::fflush(stdout);
        }
        return true;
    };

    context.runtime.runLoop(tick, &session, "throttle_step", context.sessionId);
    writePassport();

    if (context.runtime.stopped() && result.message.empty()) {
        result.message = "interrupted";
    }

    ControllerStick baseline;
    baseline.throttle = static_cast<uint16_t>(std::lround(
        result.ok ? stopClimbThr : (liftoffThr > 0 ? liftoffThr : throttle)));
    baseline.yaw = kAxisCenter;
    baseline.pitch = kAxisCenter;
    baseline.roll = kAxisCenter;
    session.setBaseline(baseline);
    session.close(false);

    writeExperimentJson(
        session.directory() + "/experiment.json",
        liftoffThr,
        stopClimbThr,
        channel.passport().normalizationVersion,
        result.ok ? "OK" : "INCOMPLETE");

    ArxOutputModel outputs[kArxOutputs];
    for (int index = 0; index < kArxOutputs; ++index) {
        outputs[index].name =
            index == 0 ? "dx_w" : (index == 1 ? "dy_h" : (index == 2 ? "log_s" : "alpha"));
    }
    double baselineValues[4] = {
        static_cast<double>(baseline.throttle),
        static_cast<double>(kAxisCenter),
        static_cast<double>(kAxisCenter),
        static_cast<double>(kAxisCenter)};
    result.model.setFromFit(
        ArxStatus::InsufficientData,
        result.ok ? "baseline_liftoff_and_stop_climb" : "throttle_step_incomplete",
        baselineValues,
        16667.0,
        0,
        outputs);
    result.model.setVisionMotion(vision);
    if (result.ok) {
        StickCalibration sticks;
        sticks.liftoffThrottle = liftoffThr;
        sticks.coarseOnsetThrottle = liftoffThr;
        sticks.stopClimbThrottle = stopClimbThr;
        sticks.hoverThrottle = stopClimbThr;
        sticks.present = true;
        sticks.hoverMeasured = true;
        if (havePrior && prior.hasRollHold()) {
            sticks.rollHoldStep = prior.sticks().rollHoldStep;
            sticks.rollHoldMax = prior.sticks().rollHoldMax;
            sticks.rollMeasured = true;
        }
        result.model.setSticks(sticks);
        result.model.setBaselineThrottle(stopClimbThr);
        result.hoverThrottle = stopClimbThr;
    }
    saveDraftModel(result.model, session.directory(), context.activeModelPath, result.ok);
    if (result.message.empty()) {
        result.message = result.ok
            ? ("liftoff=" + std::to_string(liftoffThr) + " stopClimb=" + std::to_string(stopClimbThr))
            : "did not converge";
    }
    std::printf(
        "throttle_step: done ok=%u liftoff=%u stopClimb=%u points=%u scale=%d session=%s\n",
        result.ok ? 1u : 0u,
        liftoffThr,
        stopClimbThr,
        pointFrames,
        channel.passport().normalizationVersion,
        session.directory().c_str());
    std::fflush(stdout);
    return result;
}
