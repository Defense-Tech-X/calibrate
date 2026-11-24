#pragma once

#include "Experiment.hpp"

// Pipeline step 5: score ARX forecast (NRMSE); set appliedToControl on accept.
class ValidateForecastExperiment : public Experiment {
public:
    const char* name() const override;
    ExperimentResult run(ExperimentContext& context) override;
};
