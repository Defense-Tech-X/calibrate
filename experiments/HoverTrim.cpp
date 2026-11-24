#include "experiments/HoverTrim.hpp"

#include "Experiment.hpp"
#include "Runtime.hpp"
#include "common/IpcAtomics.hpp"
#include "common/TrackReset.hpp"
#include "motion/VisualVertical.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>

namespace {

// Climb at u_lift for this long after the channel height becomes valid,
// then hold u_eq until vertical speed crosses zero. That inertial apex is h_ref.
inline constexpr uint64_t kClimbHoldUs = 1000000;
inline constexpr uint64_t kClimbLimitUs = 30000000;
inline constexpr uint64_t kCoastLimitUs = 30000000;
// One isolated probe of the active model's kH. After the apex lock, that kH
// stays fixed for kProbeUs. Every sample in the +/- kBandPx band is a hold.
// Samples on both sides of hRef are hunting. Never reaching hRef means the
// value is too small. The run does not search another kH.
inline constexpr uint64_t kProbeUs = 5000000;
inline constexpr double kBandPx = 0.5;
inline constexpr uint64_t kLogEveryUs = 500000;

std::string formatNum(double value) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.6g", value);
    return buf;
}

struct TrackResetWatch {
    HANDLE mapping = nullptr;
    TrackResetMapping* header = nullptr;
    uint64_t ticket = 0;
    bool armed = false;

    void close() {
        if (header) {
            UnmapViewOfFile(header);
            header = nullptr;
        }
        if (mapping) {
            CloseHandle(mapping);
            mapping = nullptr;
        }
    }

    bool arm() {
        mapping = OpenFileMappingW(FILE_MAP_READ | FILE_MAP_WRITE, FALSE, kTrackResetMappingName);
        if (!mapping) {
            std::printf("hover_trim: vision track reset is not available; start vision-client\n");
            std::fflush(stdout);
            return false;
        }
        header = static_cast<TrackResetMapping*>(MapViewOfFile(
            mapping,
            FILE_MAP_ALL_ACCESS,
            0,
            0,
            sizeof(TrackResetMapping)));
        if (!header || header->magic != kTrackResetMagic || header->version != kTrackResetVersion) {
            close();
            std::printf("hover_trim: vision track reset mapping has the wrong schema\n");
            std::fflush(stdout);
            return false;
        }
        ticket = incrementU64(header->request);
        armed = true;
        std::printf("reset: vision track reset requested\n");
        std::fflush(stdout);
        return true;
    }

    bool applied() const {
        return header != nullptr && loadU64(header->request) != 0 && loadU64(header->applied) == ticket;
    }
};

bool runNeutralPause(ExperimentContext& context) {
    constexpr uint64_t kSignalAtUs = 2000000;
    std::printf(
        "reset: thr=0 sticks centered for %.1fs (arm craft / prepare UI)\n",
        kCalibrateResetHoldUs / 1.0e6);
    std::fflush(stdout);

    TrackResetWatch watch;
    uint64_t phaseStartUs = 0;
    uint64_t lastLogUs = 0;
    bool interrupted = false;
    bool noted = false;
    bool ready = false;

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
        const uint64_t elapsed = nowUs - phaseStartUs;
        if (!watch.armed && elapsed >= kSignalAtUs) {
            if (!watch.arm()) {
                interrupted = true;
                return false;
            }
        }
        if (watch.armed && !noted && watch.applied()) {
            noted = true;
            std::printf("reset: vision track reset applied\n");
            std::fflush(stdout);
        }
        if (lastLogUs == 0 || nowUs - lastLogUs >= 1000000) {
            lastLogUs = nowUs;
            const double left = kCalibrateResetHoldUs > elapsed
                ? (kCalibrateResetHoldUs - elapsed) / 1.0e6
                : 0.0;
            std::printf("reset: holding neutral, %.1fs left\n", left);
            std::fflush(stdout);
        }
        if (elapsed >= kCalibrateResetHoldUs && watch.applied()) {
            ready = true;
            return false;
        }
        if (elapsed >= kCalibrateResetHoldUs + kSignalAtUs && !watch.applied()) {
            interrupted = true;
            std::printf("hover_trim: vision track reset was not applied within 2s\n");
            std::fflush(stdout);
            return false;
        }
        return true;
    };

    context.runtime.runLoop(tick, nullptr, "reset", context.sessionId);
    watch.close();
    if (interrupted || context.runtime.stopped() || !ready) {
        std::printf("reset: interrupted\n");
        std::fflush(stdout);
        return false;
    }
    std::printf("reset: done, starting experiment\n");
    std::fflush(stdout);
    return true;
}

uint16_t holdThrottle(
    double uEq,
    double kU,
    double kV,
    double kH,
    double heightPx,
    double hRefPx,
    double speedPxPerSec) {
    const double command =
        uEq - (kH / kU) * (heightPx - hRefPx) - (kV / kU) * speedPxPerSec;
    long rounded = std::lround(command);
    if (rounded < 0) {
        rounded = 0;
    }
    if (rounded > static_cast<long>(kAxisMax)) {
        rounded = static_cast<long>(kAxisMax);
    }
    return static_cast<uint16_t>(rounded);
}

void writeExperimentJson(
    const std::string& path,
    bool ok,
    double uEq,
    double uLift,
    double kU,
    double kV,
    double kH,
    bool kHWritten,
    double hRefPx,
    bool heightLocked,
    double spanPx,
    double hBelowPx,
    double hAbovePx) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        return;
    }
    out << "{\n  \"experiment\": \"hover_trim\",\n  \"status\": \""
        << (ok ? "OK" : "INCOMPLETE") << "\",\n  \"uEq\": " << formatNum(uEq)
        << ",\n  \"uLift\": " << formatNum(uLift)
        << ",\n  \"kU\": " << formatNum(kU)
        << ",\n  \"kV\": " << formatNum(kV)
        << ",\n  \"kH\": " << (kHWritten ? formatNum(kH) : "0")
        << ",\n  \"kHMeasured\": " << (kHWritten ? "true" : "false")
        << ",\n  \"hRefPx\": " << formatNum(hRefPx)
        << ",\n  \"heightLocked\": " << (heightLocked ? "true" : "false")
        << ",\n  \"spanPx\": " << formatNum(spanPx)
        << ",\n  \"hBelowPx\": " << formatNum(hBelowPx)
        << ",\n  \"hAbovePx\": " << formatNum(hAbovePx)
        << "\n}\n";
}

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

    const VerticalDynamics seeded = prior.vertical();
    if (!(seeded.kU > 0.0) || !(seeded.kV > 0.0)) {
        result.message = "kU and kV required; vertical block missing on the active model";
        return result;
    }
    const StickCalibration sticks = prior.sticks();
    if (!(sticks.liftoffThrottle > 0.0) || !(sticks.stopClimbThrottle > 0.0)) {
        result.message = "liftoffThrottle + stopClimbThrottle required; run throttle_step first";
        return result;
    }

    const double uLift = sticks.liftoffThrottle;
    const double uEq = sticks.stopClimbThrottle;
    const double kU = seeded.kU;
    const double kV = seeded.kV;
    if (!(seeded.kH > 0.0)) {
        result.message = "vertical.kH required on the active model";
        return result;
    }
    const double kH = seeded.kH;
    const uint16_t climbThrottle = static_cast<uint16_t>(std::lround(uLift));
    const uint16_t eqThrottle = static_cast<uint16_t>(std::lround(uEq));

    if (!runNeutralPause(context)) {
        result.message = "interrupted during reset";
        return result;
    }

    context.session.open(context.sessionId, context.sessionsRoot, "hover_trim");

    enum class Phase { Climb, Coast, Hunt, Done };
    Phase phase = Phase::Climb;

    uint16_t throttle = climbThrottle;
    bool aboveRef = false;
    bool belowRef = false;
    bool inBandAll = true;
    bool haveProbeSample = false;
    bool measureHave = false;
    double measureMinH = 0.0;
    double measureMaxH = 0.0;
    double hRefPx = 0.0;
    bool heightLocked = false;
    uint64_t climbStartUs = 0;
    uint64_t airborneUs = 0;
    uint64_t coastStartUs = 0;
    uint64_t huntStartUs = 0;
    double spanPx = 0.0;
    bool havePrevSpeed = false;
    double prevSpeed = 0.0;
    uint64_t lastLogUs = 0;
    uint64_t lastSampleUs = 0;
    uint64_t lastSampleSeq = 0;
    bool haveSampleStamp = false;
    double lastHeight = 0.0;
    double lastSpeed = 0.0;
    bool haveHeight = false;
    bool haveSpeed = false;

    VisualVerticalChannel channel;
    int passportVersion = -1;
    size_t passportPoints = 0;
    const std::string channelPath = context.session.directory() + "/visual_channel.jsonl";
    const std::string passportPath = context.session.directory() + "/visual_scale.json";
    std::ofstream channelLog(channelPath, std::ios::binary);
    auto writePassport = [&]() {
        std::ofstream passport(passportPath, std::ios::binary);
        if (passport) {
            passport << channel.passportJson();
        }
    };

    auto finish = [&](bool ok, const char* why) {
        result.ok = ok;
        result.message = why;
        phase = Phase::Done;
        if (ok) {
            result.hoverThrottle = uEq;
            VerticalDynamics vertical = seeded;
            vertical.kH = kH;
            vertical.spanPx = spanPx;
            vertical.kHMeasured = true;
            vertical.present = true;
            result.model = prior;
            result.model.setVertical(vertical);
            std::printf(
                "hover_trim: OK kH=%s span=%s hBelow=%s hAbove=%s hRef=%s\n",
                formatNum(kH).c_str(),
                formatNum(spanPx).c_str(),
                formatNum(measureMinH).c_str(),
                formatNum(measureMaxH).c_str(),
                formatNum(hRefPx).c_str());
        } else {
            std::printf("hover_trim: INCOMPLETE %s\n", why);
        }
        std::fflush(stdout);
    };

    std::printf(
        "hover_trim: climb at uLift=%s for %.1fs after height is valid, "
        "then uEq=%s until speed crosses 0; that apex is hRef\n"
        "hover_trim: u = uEq - (kH/kU)*(h-hRef) - (kV/kU)*v; "
        "kU=%s kV=%s one isolated kH=%s for %.0fs\n"
        "hover_trim: session=%s\n",
        formatNum(uLift).c_str(),
        kClimbHoldUs / 1.0e6,
        formatNum(uEq).c_str(),
        formatNum(kU).c_str(),
        formatNum(kV).c_str(),
        formatNum(kH).c_str(),
        kProbeUs / 1.0e6,
        context.session.directory().c_str());
    std::fflush(stdout);

    auto tick = [&](uint64_t nowUs,
                    const ControllerStick&,
                    const PointCloud* cloud,
                    const PointShift*,
                    const ObservationSample*,
                    bool) -> bool {
        if (context.runtime.stopped()) {
            finish(false, "interrupted");
            return false;
        }
        if (climbStartUs == 0) {
            climbStartUs = nowUs;
        }

        bool newSample = false;
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
            newSample = !haveSampleStamp ||
                report->measurementTimestampUs != lastSampleUs ||
                report->sequence != lastSampleSeq;
            if (newSample) {
                haveSampleStamp = true;
                lastSampleUs = report->measurementTimestampUs;
                lastSampleSeq = report->sequence;
            }
            if (report->heightValid) {
                haveHeight = true;
                lastHeight = report->visualHeightPx;
            }
            if (report->speedValid) {
                haveSpeed = true;
                lastSpeed = report->visualVerticalSpeedPxPerSecond;
            }
        }

        if (phase == Phase::Climb) {
            throttle = climbThrottle;
            if (airborneUs == 0) {
                if (newSample && report != nullptr && report->heightValid) {
                    airborneUs = nowUs;
                    std::printf(
                        "hover_trim: climb height valid, hold uLift for %.1fs\n",
                        kClimbHoldUs / 1.0e6);
                    std::fflush(stdout);
                } else if (nowUs - climbStartUs >= kClimbLimitUs) {
                    finish(false, "climb did not start");
                    return false;
                }
            } else if (nowUs - airborneUs >= kClimbHoldUs) {
                throttle = eqThrottle;
                phase = Phase::Coast;
                coastStartUs = nowUs;
                std::printf("hover_trim: coast at uEq=%u until speed crosses 0\n", eqThrottle);
                std::fflush(stdout);
            }
        }

        if (phase == Phase::Coast) {
            throttle = eqThrottle;
            const bool crossedZero = newSample && report != nullptr &&
                report->heightValid && report->speedValid &&
                havePrevSpeed && prevSpeed > 0.0 &&
                report->visualVerticalSpeedPxPerSecond <= 0.0;
            if (crossedZero) {
                hRefPx = report->visualHeightPx;
                heightLocked = true;
                phase = Phase::Hunt;
                huntStartUs = nowUs;
                std::printf(
                    "hover_trim: HEIGHT LOCK hRef=%s speed=%s thr=%u kH=%s\n",
                    formatNum(hRefPx).c_str(),
                    formatNum(report->visualVerticalSpeedPxPerSecond).c_str(),
                    throttle,
                    formatNum(kH).c_str());
                std::fflush(stdout);
            } else if (coastStartUs != 0 && nowUs - coastStartUs >= kCoastLimitUs) {
                finish(false, "speed did not reach zero");
                return false;
            }
        }

        if (newSample && report != nullptr && report->speedValid) {
            prevSpeed = report->visualVerticalSpeedPxPerSecond;
            havePrevSpeed = true;
        }

        if (phase == Phase::Hunt) {
            if (huntStartUs == 0) {
                huntStartUs = nowUs;
            }
            if (newSample && report != nullptr && report->heightValid) {
                const double height = report->visualHeightPx;
                const double error = height - hRefPx;
                haveProbeSample = true;
                if (height > hRefPx) {
                    aboveRef = true;
                }
                if (height < hRefPx) {
                    belowRef = true;
                }
                if (std::fabs(error) > kBandPx) {
                    inBandAll = false;
                }
                if (!measureHave) {
                    measureHave = true;
                    measureMinH = height;
                    measureMaxH = height;
                } else {
                    if (height < measureMinH) {
                        measureMinH = height;
                    }
                    if (height > measureMaxH) {
                        measureMaxH = height;
                    }
                }
                if (report->speedValid) {
                    throttle = holdThrottle(
                        uEq,
                        kU,
                        kV,
                        kH,
                        height,
                        hRefPx,
                        report->visualVerticalSpeedPxPerSecond);
                }
            }
            if (haveProbeSample && nowUs - huntStartUs >= kProbeUs) {
                spanPx = measureMaxH - measureMinH;
                if (inBandAll) {
                    std::printf(
                        "hover_trim: kH=%s stayed inside +-0.5 px of h_ref, this value is good\n",
                        formatNum(kH).c_str());
                    std::fflush(stdout);
                    finish(true, ("kH=" + formatNum(kH)).c_str());
                } else if (aboveRef && belowRef) {
                    std::printf(
                        "hover_trim: kH=%s oscillated above and below h_ref, decrease k_h and restart the test\n",
                        formatNum(kH).c_str());
                    std::fflush(stdout);
                    finish(false, "decrease kH and restart");
                } else if (!aboveRef) {
                    std::printf(
                        "hover_trim: kH=%s did not climb to h_ref, increase k_h and restart the test\n",
                        formatNum(kH).c_str());
                    std::fflush(stdout);
                    finish(false, "increase kH and restart");
                } else {
                    std::printf(
                        "hover_trim: kH=%s climbed above the h_ref band, decrease k_h and restart the test\n",
                        formatNum(kH).c_str());
                    std::fflush(stdout);
                    finish(false, "decrease kH and restart");
                }
                return false;
            }
        }



        ControllerStick command;
        command.throttle = throttle;
        command.yaw = kAxisCenter;
        command.pitch = kAxisCenter;
        command.roll = kAxisCenter;
        command.flightMode = kFlightModeAngle;
        context.runtime.writeStick(command);

        if (lastLogUs == 0 || nowUs - lastLogUs >= kLogEveryUs) {
            lastLogUs = nowUs;
            const char* phaseName = phase == Phase::Hunt
                ? "hunt"
                : (phase == Phase::Coast ? "coast" : "climb");
            std::printf("hover_trim: phase=%s thr=%u kH=%s ", phaseName, throttle, formatNum(kH).c_str());
            if (haveHeight) {
                std::printf("height=%s ", formatNum(lastHeight).c_str());
            } else {
                std::printf("height=invalid ");
            }
            if (haveSpeed) {
                std::printf("speed=%s ", formatNum(lastSpeed).c_str());
            } else {
                std::printf("speed=invalid ");
            }
            if (heightLocked) {
                std::printf("err=%s", formatNum(lastHeight - hRefPx).c_str());
            }
            std::printf("\n");
            std::fflush(stdout);
        }
        return phase != Phase::Done;
    };

    context.runtime.runLoop(tick, &context.session, "hover_trim", context.sessionId);
    writePassport();

    if (context.runtime.stopped() && result.message.empty()) {
        result.message = "interrupted";
    }
    if (result.message.empty()) {
        result.message = "did not lock a height";
    }

    ControllerStick baseline = stick;
    baseline.throttle = static_cast<uint16_t>(std::lround(uEq));
    baseline.yaw = kAxisCenter;
    baseline.pitch = kAxisCenter;
    baseline.roll = kAxisCenter;
    context.session.setBaseline(baseline);
    context.session.close(false);

    writeExperimentJson(
        context.session.directory() + "/experiment.json",
        result.ok,
        uEq,
        uLift,
        kU,
        kV,
        result.ok ? kH : 0.0,
        result.ok,
        hRefPx,
        heightLocked,
        result.ok ? spanPx : 0.0,
        result.ok ? measureMinH : 0.0,
        result.ok ? measureMaxH : 0.0);

    if (result.ok) {
        saveDraftModel(result.model, context.session.directory(), context.activeModelPath, true);
    } else {
        saveDraftModel(prior, context.session.directory(), context.activeModelPath, false);
    }

    std::printf(
        "hover_trim: done ok=%u kH=%s measured=%u hRef=%s span=%s\n",
        result.ok ? 1u : 0u,
        formatNum(result.ok ? kH : 0.0).c_str(),
        result.ok ? 1u : 0u,
        formatNum(hRefPx).c_str(),
        formatNum(result.ok ? spanPx : 0.0).c_str());
    std::fflush(stdout);
    return result;
}
