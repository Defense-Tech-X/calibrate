#pragma once

#include "Experiment.hpp"

class YawPulseExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};

class MultiAxisExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
