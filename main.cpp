#include "Experiment.hpp"
#include "Runtime.hpp"
#include "experiments/PitchPulse.hpp"
#include "experiments/RollPulse.hpp"
#include "experiments/Stubs.hpp"
#include "experiments/HoverTrim.hpp"
#include "experiments/ThrottleStep.hpp"
#include "experiments/ValidateForecast.hpp"
#include "ControllerProtocol.hpp"
#include "identify/ArxModel.hpp"

#include <cstdio>
#include <cstring>
#include <exception>
#include <memory>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::printf(
        "Usage: calibrate <experiment>\n"
        "Live pipeline (run in order):\n"
        "  throttle_step      1) visual height/speed channel -> liftoff + stopClimb\n"
        "  hover_trim         2) apex lock + cascade hold; still 5s -> hoverThrottle\n"
        "  roll_pulse         3) hold altitude SoftCeil; measure rollHold* (no LAT)\n"
        "  pitch_pulse        4) hold altitude+LAT; measure pitch stick\n"
        "  validate_forecast  5) score ARX forecast; set appliedToControl\n"
        "Other:\n"
        "  yaw_pulse          (stub)\n"
        "  multi_axis         (stub)\n"
        "  fit_synthetic      offline smoke: fit + promote active model\n");
}

std::unique_ptr<Experiment> makeExperiment(const std::string& name) {
    if (name == "throttle_step") {
        return std::make_unique<ThrottleStepExperiment>();
    }
    if (name == "hover_trim") {
        return std::make_unique<HoverTrimExperiment>();
    }
    if (name == "roll_pulse") {
        return std::make_unique<RollPulseExperiment>();
    }
    if (name == "pitch_pulse") {
        return std::make_unique<PitchPulseExperiment>();
    }
    if (name == "validate_forecast") {
        return std::make_unique<ValidateForecastExperiment>();
    }
    if (name == "yaw_pulse") {
        return std::make_unique<YawPulseExperiment>();
    }
    if (name == "multi_axis") {
        return std::make_unique<MultiAxisExperiment>();
    }
    return nullptr;
}

int runSyntheticFit(const std::string& activePath) {
    constexpr int n = 600;
    std::vector<ArxRow> rows(static_cast<size_t>(n));
    const double baseline[4] = { 14000, kAxisCenter, kAxisCenter, kAxisCenter };
    double y[4] = {};
    for (int index = 0; index < n; ++index) {
        ArxRow& row = rows[static_cast<size_t>(index)];
        row.timeUs = index * 20000.0;
        // Persistent PRBS-like excitation on every axis so MIMO ARX is identifiable.
        row.u[0] = ((index / 17) % 2 == 0 ? 1.0 : -1.0) * (180.0 + (index % 5) * 10.0);
        row.u[1] = ((index / 23) % 2 == 0 ? 1.0 : -1.0) * (120.0 + (index % 3) * 15.0);
        row.u[2] = ((index / 19) % 2 == 0 ? 1.0 : -1.0) * (160.0 + (index % 4) * 12.0);
        row.u[3] = ((index / 13) % 2 == 0 ? 1.0 : -1.0) * (200.0 + (index % 7) * 8.0);
        // True plant: first-order per channel with light cross-coupling.
        y[0] = 0.70 * y[0] + 0.00040 * row.u[3] + 0.00005 * row.u[2];
        y[1] = 0.72 * y[1] + 0.00035 * row.u[0] + 0.00004 * row.u[2];
        y[2] = 0.75 * y[2] + 0.00020 * row.u[0] + 0.00003 * row.u[3];
        y[3] = 0.68 * y[3] + 0.00030 * row.u[1] + 0.00002 * row.u[3];
        for (int output = 0; output < 4; ++output) {
            row.y[output] = y[output];
        }
    }

    ArxModel model = fitArxModel(rows, baseline);
    CreateDirectoryA("plant", nullptr);
    CreateDirectoryA("plant/models", nullptr);
    CreateDirectoryA("calibrate", nullptr);
    CreateDirectoryA("calibrate/sessions", nullptr);
    CreateDirectoryA("calibrate/sessions/synthetic", nullptr);
    const bool ok = model.status() == ArxStatus::Ok;
    model.setAppliedToControl(ok);
    model.save("calibrate/sessions/synthetic/model.json");
    model.save(activePath);

    // Round-trip load + forecast smoke.
    ArxModel loaded;
    const bool loadedOk = loaded.load(activePath) && loaded.readyForControl();
    std::vector<std::array<double, kArxInputs>> future(8);
    const auto forecast = loaded.forecast(8, rows, future);
    std::printf(
        "fit_synthetic: status=%d applied=%d loaded=%d forecastSteps=%zu path=%s\n",
        static_cast<int>(model.status()),
        model.appliedToControl() ? 1 : 0,
        loadedOk ? 1 : 0,
        forecast.size(),
        activePath.c_str());
    return loadedOk ? 0 : 2;
}

} // namespace

// calibrate: control model from the vision point cloud and stick commands.
// task: closed-loop flight once appliedToControl is set.
// throttle_step: visual vertical channel (px, px/s) for a later altitude-law fit.
// Pulse experiments reuse accepted altitude/LAT holds from calibrate/hold/*.
int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage();
        return 1;
    }
    const std::string name = argv[1];
    const std::string activePath = ArxModel::defaultActivePath();

    if (name == "fit_synthetic") {
        return runSyntheticFit(activePath);
    }

    auto experiment = makeExperiment(name);
    if (!experiment) {
        printUsage();
        return 1;
    }

    SafeRuntime runtime;
    SessionLog session;
    try {
        std::printf("calibrate: opening vision points, observation, HID feeder...\n");
        std::fflush(stdout);
        runtime.setupVisionAndController();
        std::printf("calibrate: channels ready, starting %s (Ctrl+C parks throttle and stops)\n", name.c_str());
        std::fflush(stdout);
        ExperimentContext context{
            runtime,
            session,
            qpcNowUs(),
            "calibrate/sessions",
            activePath };
        const ExperimentResult result = experiment->run(context);
        if (runtime.stopped()) {
            std::printf("%s: interrupted (%s)\n", experiment->name(), result.message.c_str());
        } else {
            std::printf("%s: %s\n", experiment->name(), result.message.c_str());
        }
        std::fflush(stdout);
        runtime.shutdown();
        return result.ok ? 0 : 2;
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "%s\n", ex.what());
        std::fflush(stderr);
        runtime.shutdown();
        return 1;
    }
}
