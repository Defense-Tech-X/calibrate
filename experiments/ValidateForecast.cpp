#include "experiments/ValidateForecast.hpp"

#include "Experiment.hpp"
#include "Runtime.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <vector>

const char* ValidateForecastExperiment::name() const {
    return "validate_forecast";
}

ExperimentResult ValidateForecastExperiment::run(ExperimentContext& context) {
    ExperimentResult result;
    ArxModel model;
    ControllerStick hover;
    if (!loadBaselineModel(context.activeModelPath, model, hover) || model.status() != ArxStatus::Ok) {
        result.message = "active model missing or not OK; run roll_pulse/pitch_pulse first";
        return result;
    }

    if (!runResetPhase(context)) {
        result.message = "interrupted during reset";
        return result;
    }

    context.session.open(context.sessionId, context.sessionsRoot, "validate_forecast");
    context.session.setBaseline(hover);

    std::vector<ArxRow> live;
    enum class Phase { Settle, Pulse, Done };
    Phase phase = Phase::Settle;
    uint64_t phaseStart = 0;
    uint64_t lastLogUs = 0;
    bool locked = false;
    const uint64_t settleUs = 1000000;
    const uint64_t pulseUs = 300000;
    const uint64_t logEveryUs = 1000000;

    std::printf(
        "validate_forecast: step5 - score ARX forecast (NRMSE); set appliedToControl\n"
        "validate_forecast: hoverThr=%u, settle=%.1fs, roll pulse=%.2fs, session=%s\n",
        hover.throttle,
        settleUs / 1.0e6,
        pulseUs / 1.0e6,
        context.session.directory().c_str());
    std::fflush(stdout);

    auto tick = [&](uint64_t nowUs,
                    const ControllerStick& current,
                    const PointCloud*,
                    const PointShift*,
                    const ObservationSample* observation,
                    bool referenceReady) -> bool {
        if (context.runtime.stopped()) {
            result.message = "interrupted";
            return false;
        }
        if (!locked) {
            context.runtime.observation().requestLock(context.sessionId, context.session.keyframePath());
            ControllerStick command = hover;
            context.runtime.writeStick(command);
            if (referenceReady) {
                locked = true;
                phase = Phase::Settle;
                phaseStart = nowUs;
                std::printf("validate_forecast: reference locked, settling...\n");
                std::fflush(stdout);
            } else if (lastLogUs == 0 || nowUs - lastLogUs >= logEveryUs) {
                lastLogUs = nowUs;
                std::printf("validate_forecast: waiting for reference lock...\n");
                std::fflush(stdout);
            }
            return true;
        }

        ControllerStick command = hover;
        const char* phaseName = "settle";
        if (phase == Phase::Settle) {
            if (nowUs - phaseStart >= settleUs) {
                phase = Phase::Pulse;
                phaseStart = nowUs;
                std::printf("validate_forecast: roll pulse +200\n");
                std::fflush(stdout);
            }
        } else if (phase == Phase::Pulse) {
            phaseName = "pulse";
            command.roll = static_cast<uint16_t>(std::clamp(static_cast<int>(kAxisCenter) + 200, 0, static_cast<int>(kAxisMax)));
            if (nowUs - phaseStart >= pulseUs) {
                phase = Phase::Done;
                std::printf("validate_forecast: capture done, scoring forecast (%zu samples)\n", live.size());
                std::fflush(stdout);
            }
        } else {
            return false;
        }

        if (observation != nullptr && referenceReady) {
            ArxRow row;
            row.timeUs = static_cast<double>(observation->captureTimestampUs);
            for (int index = 0; index < 4; ++index) {
                row.y[index] = observation->y[index];
            }
            row.u[0] = static_cast<double>(command.throttle) - model.baseline()[0];
            row.u[1] = static_cast<double>(command.yaw) - model.baseline()[1];
            row.u[2] = static_cast<double>(command.pitch) - model.baseline()[2];
            row.u[3] = static_cast<double>(command.roll) - model.baseline()[3];
            live.push_back(row);
        }

        if (command.throttle != current.throttle || command.roll != current.roll || command.pitch != current.pitch ||
            command.yaw != current.yaw) {
            context.runtime.writeStick(command);
        }

        if (lastLogUs == 0 || nowUs - lastLogUs >= logEveryUs) {
            lastLogUs = nowUs;
            std::printf(
                "validate_forecast: phase=%s samples=%zu thr=%u roll=%u\n",
                phaseName,
                live.size(),
                command.throttle,
                command.roll);
            std::fflush(stdout);
        }
        return true;
    };

    context.runtime.runLoop(tick, &context.session, "validate_forecast", context.sessionId);
    context.session.close(false);

    if (live.size() < 40) {
        result.message = "too few validation samples";
        return result;
    }

    const int historyCount = static_cast<int>(live.size()) / 2;
    std::vector<ArxRow> history(live.begin(), live.begin() + historyCount);
    std::vector<std::array<double, kArxInputs>> futureU;
    for (size_t index = static_cast<size_t>(historyCount); index < live.size(); ++index) {
        futureU.push_back({ live[index].u[0], live[index].u[1], live[index].u[2], live[index].u[3] });
    }
    const auto predicted = model.forecast(static_cast<int>(futureU.size()), history, futureU);
    double sse = 0.0;
    double energy = 0.0;
    for (size_t index = 0; index < predicted.size(); ++index) {
        for (int output = 0; output < kArxOutputs; ++output) {
            const double y = live[static_cast<size_t>(historyCount) + index].y[output];
            const double err = predicted[index][static_cast<size_t>(output)] - y;
            sse += err * err;
            energy += y * y;
        }
    }
    const double nrmse = energy > 1.0e-12 ? std::sqrt(sse / energy) : sse;
    const bool accept = nrmse < 1.0;
    model.setAppliedToControl(accept);
    saveDraftModel(model, context.session.directory(), context.activeModelPath, accept);

    std::ofstream meta(context.session.directory() + "/experiment.json", std::ios::binary);
    if (meta) {
        meta << "{\n  \"experiment\": \"validate_forecast\",\n  \"nrmse\": " << nrmse
             << ",\n  \"accepted\": " << (accept ? "true" : "false") << "\n}\n";
    }

    result.ok = accept;
    result.model = model;
    result.hoverThrottle = model.baseline()[0];
    result.message = accept ? ("accepted nrmse=" + std::to_string(nrmse))
                            : ("rejected nrmse=" + std::to_string(nrmse));
    std::printf("validate_forecast: %s\n", result.message.c_str());
    return result;
}
