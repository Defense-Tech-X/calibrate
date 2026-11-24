#pragma once

#include "Experiment.hpp"

// Pipeline step 4: hold altitude + LAT; measure pitch stick response (+ ARX draft).
class PitchPulseExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
