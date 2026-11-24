#include "experiments/Stubs.hpp"

const char* YawPulseExperiment::name() const {
    return "yaw_pulse";
}

ExperimentResult YawPulseExperiment::run(ExperimentContext&) {
    ExperimentResult result;
    result.message = "stub: yaw_pulse not implemented yet";
    return result;
}

const char* MultiAxisExperiment::name() const {
    return "multi_axis";
}

ExperimentResult MultiAxisExperiment::run(ExperimentContext&) {
    ExperimentResult result;
    result.message = "stub: multi_axis not implemented yet";
    return result;
}
