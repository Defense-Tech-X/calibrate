#pragma once

#include "Experiment.hpp"

// Pipeline step 3: hold altitude; measure sticks.rollHoldStep/Max (+ ARX draft).
class RollPulseExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
